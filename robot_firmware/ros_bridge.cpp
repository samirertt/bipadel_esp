// ============================================================
// ros_bridge.cpp
// ============================================================
// Implementation of the ROS 2 serial bridge.
//
// PROTOCOL OVERVIEW
// -----------------
// The ROS 2 DiffDrive hardware interface communicates over a
// simple ASCII serial line:
//
//   'b'              → respond with baud rate integer
//   'e'              → respond with "<left_mdeg> <right_mdeg>\n"
//   'r'              → reset encoder positions; respond "OK\n"
//   'm <L> <R>\n'    → set wheel velocities in milli-rad/s
//
// Units coming from ROS are intentionally in milli-rad/s so that
// integer-only serial parsers avoid floating-point formatting.
// This module converts them back to float rad/s immediately.
//
// COMMAND NORMALISATION
// ---------------------
// Rather than adding a separate ROS command path in the main
// control loop, the bridge converts wheel rad/s targets into the
// same [-1,1] forward/turn normalised commands that the PS4
// joystick produces.  This keeps the LQR / control_manager
// entirely agnostic of the input source.
// ============================================================

#include "ros_bridge.h"
#include "config.h"
#include "can_bus.h"
#include "utils.h"
#include <stdlib.h>
#include <math.h>

// ============================================================
// Serial parser state
// ============================================================
// The parser is a simple state-machine that reads one character
// at a time.  Fields:
//   cmd   – the opcode character received so far
//   arg   – which argument slot we are filling (0=opcode, 1=arg1, 2=arg2)
//   indx  – write index into the current argv buffer
//   argv1 – first argument string (null-terminated)
//   argv2 – second argument string (null-terminated)
static char s_cmd    = 0;
static int  s_arg    = 0;
static int  s_indx   = 0;
static char s_argv1[20];
static char s_argv2[20];

// ============================================================
// Normalised command outputs (written by apply_wheel_targets,
// read by ros_bridge_get_commands)
// ============================================================
static float         s_forward_cmd  = 0.0f;
static float         s_turn_cmd     = 0.0f;
static bool          s_has_ros_cmd  = false;

// Raw wheel targets (stored for demo-feedback integration)
static float         s_left_cmd_rad_s  = 0.0f;
static float         s_right_cmd_rad_s = 0.0f;

// ============================================================
// Watchdog
// ============================================================
static unsigned long s_last_cmd_ms  = 0;
static unsigned long s_timeout_ms   = ROS_CMD_TIMEOUT_MS;

// ============================================================
// Encoder reset flag
// ============================================================
// Set when ROS sends 'r'; cleared by ros_bridge_clear_reset_request().
static bool s_reset_requested = false;

// ============================================================
// Demo feedback state  (compiled in only when DEMO flag is set)
// ============================================================
// When real CAN motor feedback is not available (bench testing),
// the bridge can simulate encoder positions by integrating the
// commanded wheel velocities over time.
#if ROS_BRIDGE_DEMO_FEEDBACK
static float         s_demo_left_pos_deg   = 0.0f;
static float         s_demo_right_pos_deg  = 0.0f;
static float         s_demo_left_vel_dps   = 0.0f;
static float         s_demo_right_vel_dps  = 0.0f;
static unsigned long s_demo_last_update_ms = 0;
#endif

// ============================================================
// Internal helpers
// ============================================================

// Reset the serial parser back to its idle state.
static void reset_parser() {
  s_cmd      = 0;
  s_arg      = 0;
  s_indx     = 0;
  s_argv1[0] = '\0';
  s_argv2[0] = '\0';
}

#if ROS_BRIDGE_DEMO_FEEDBACK
// Integrate the current wheel velocity commands to advance the
// simulated encoder positions.  Called each loop and before any
// 'e' response so ROS always gets fresh-ish data.
static void update_demo_feedback() {
  unsigned long now = millis();

  // On first call, just initialise the timestamp and return.
  if (s_demo_last_update_ms == 0) {
    s_demo_last_update_ms = now;
    return;
  }

  float dt = (now - s_demo_last_update_ms) * 0.001f;
  s_demo_last_update_ms = now;
  if (dt <= 0.0f) return;

  // Convert commanded joint rad/s to deg/s, then integrate.
  s_demo_left_vel_dps  = rad2deg(s_left_cmd_rad_s);
  s_demo_right_vel_dps = rad2deg(s_right_cmd_rad_s);
  s_demo_left_pos_deg  += s_demo_left_vel_dps  * dt;
  s_demo_right_pos_deg += s_demo_right_vel_dps * dt;
}

// Build a MotorFeedback struct from the simulated state so the
// rest of the code can call get_demo_feedback() uniformly.
static MotorFeedback get_demo_feedback() {
  MotorFeedback fb{};
  fb.left_pos_deg  = s_demo_left_pos_deg;
  fb.right_pos_deg = s_demo_right_pos_deg;
  fb.left_vel_dps  = s_demo_left_vel_dps;
  fb.right_vel_dps = s_demo_right_vel_dps;
  return fb;
}
#endif  // ROS_BRIDGE_DEMO_FEEDBACK

// Convert raw wheel targets (rad/s at the joint) into the
// normalised [-1,1] forward / turn values consumed by the
// main control loop.
//
// The PS4 path already produces values in this range, so sharing
// the same representation lets control_manager.cpp remain unaware
// of whether a human or ROS is driving.
static void apply_wheel_targets_rad_s(float left_rad_s, float right_rad_s) {
  // Store raw targets (used by demo feedback integrator)
  s_left_cmd_rad_s  = left_rad_s;
  s_right_cmd_rad_s = right_rad_s;

  // Decompose into forward (average) and turn (difference) components.
  float avg_rad_s  = 0.5f * (left_rad_s  + right_rad_s);
  float diff_rad_s = 0.5f * (right_rad_s - left_rad_s);

  // Normalise by the maximum expected wheel speed.
  float forward = avg_rad_s  / ROS_BRIDGE_MAX_WHEEL_RAD_S;
  float turn    = diff_rad_s / ROS_BRIDGE_MAX_WHEEL_RAD_S;

  // Clamp to valid range and apply dead-band.
  forward = clampf(forward, -1.0f,  1.0f);
  turn    = clampf(turn,    -1.0f,  1.0f);
  if (fabs(forward) < ROS_BRIDGE_CMD_DEADBAND) forward = 0.0f;
  if (fabs(turn)    < ROS_BRIDGE_CMD_DEADBAND) turn    = 0.0f;

  s_forward_cmd = forward;
  s_turn_cmd    = turn;
  s_has_ros_cmd = true;
  s_last_cmd_ms = millis();
}

// Execute a fully-parsed command.  argv1 and argv2 are already
// null-terminated at this point.
static void run_command() {
  long arg1 = atol(s_argv1);
  long arg2 = atol(s_argv2);

  switch (s_cmd) {

    // 'b' – report baud rate so ROS can verify the connection.
    case GET_BAUDRATE:
      Serial.println(ROS_BRIDGE_BAUD);
      break;

    // 'e' – report wheel encoder positions in milli-degrees.
    // ROS expects two space-separated integers on a single line.
    case READ_ENCODERS: {
#if ROS_BRIDGE_DEMO_FEEDBACK
      update_demo_feedback();
      MotorFeedback fb = get_demo_feedback();
#else
      MotorFeedback fb = can_bus_get_feedback();
#endif
      // Multiply degrees by 1000 → milli-degrees integer.
      Serial.print((long)(fb.left_pos_deg  * 1000.0f));
      Serial.print(' ');
      Serial.println((long)(fb.right_pos_deg * 1000.0f));
      break;
    }

    // 'r' – zero the encoder reference.
    // Raises a flag so the main loop can call estimator_init() or
    // equivalent.  Demo state is also zeroed if active.
    case RESET_ENCODERS:
      s_reset_requested = true;
#if ROS_BRIDGE_DEMO_FEEDBACK
      s_demo_left_pos_deg   = 0.0f;
      s_demo_right_pos_deg  = 0.0f;
      s_demo_left_vel_dps   = 0.0f;
      s_demo_right_vel_dps  = 0.0f;
      s_left_cmd_rad_s      = 0.0f;
      s_right_cmd_rad_s     = 0.0f;
#endif
      Serial.println("OK");
      break;

    // 'm <L> <R>' – set wheel velocities.
    // ROS sends values in milli-rad/s; divide by 1000 to get rad/s.
    case MOTOR_SPEEDS: {
      float left_rad_s  = (float)arg1 / 1000.0f;
      float right_rad_s = (float)arg2 / 1000.0f;
      apply_wheel_targets_rad_s(left_rad_s, right_rad_s);
      Serial.println("OK");
      break;
    }

    default:
      Serial.println("Invalid Command");
      break;
  }
}

// ============================================================
// Public API implementation
// ============================================================

bool ros_bridge_init(unsigned long baudrate) {
  // Serial should already be started from setup(); calling begin()
  // again with the same baud rate is harmless but we do it here so
  // the bridge module is fully self-contained if moved.
  Serial.begin(baudrate);
  delay(200);

  s_last_cmd_ms = millis();
  reset_parser();

#if ROS_BRIDGE_DEMO_FEEDBACK
  // Seed the demo integrator timestamp so the first dt is not huge.
  s_demo_last_update_ms = millis();
  s_demo_left_pos_deg   = 0.0f;
  s_demo_right_pos_deg  = 0.0f;
  s_demo_left_vel_dps   = 0.0f;
  s_demo_right_vel_dps  = 0.0f;
#endif

  return true;
}

void ros_bridge_set_timeout_ms(unsigned long timeout_ms) {
  s_timeout_ms = timeout_ms;
}

void ros_bridge_update() {
#if ROS_BRIDGE_DEMO_FEEDBACK
  // Advance the simulated encoder positions even when not responding
  // to an 'e' query, so velocity estimates stay smooth.
  update_demo_feedback();
#endif

  // --- Serial character parser ---
  // We process all available bytes in a single call so that the main
  // loop's timing is not affected by partially-received commands.
  while (Serial.available() > 0) {
    char chr = static_cast<char>(Serial.read());

    if (chr == '\r' || chr == '\n') {
      // End of line: execute the command if we have a valid opcode.
      if (s_cmd != 0) {
        // Null-terminate whichever argument was being filled.
        if      (s_arg == 1) s_argv1[s_indx] = '\0';
        else if (s_arg == 2) s_argv2[s_indx] = '\0';

        run_command();
        reset_parser();
      }
    }
    else if (chr == ' ') {
      // Space advances to the next argument slot.
      if (s_arg == 0) {
        s_arg  = 1;
        s_indx = 0;
      } else if (s_arg == 1) {
        s_argv1[s_indx] = '\0';
        s_arg  = 2;
        s_indx = 0;
      }
      // A third argument would be silently ignored.
    }
    else {
      // Accumulate character into the appropriate slot.
      if (s_arg == 0) {
        s_cmd = chr;  // First non-space char is the opcode.
      } else if (s_arg == 1) {
        if (s_indx < (int)sizeof(s_argv1) - 1)
          s_argv1[s_indx++] = chr;
      } else if (s_arg == 2) {
        if (s_indx < (int)sizeof(s_argv2) - 1)
          s_argv2[s_indx++] = chr;
      }
    }
  }

  // --- Watchdog ---
  // If no 'm' command has been received recently, zero the outputs
  // so the robot comes to a stop rather than continuing with a stale
  // command when the ROS host crashes or the cable is unplugged.
  if ((millis() - s_last_cmd_ms) > s_timeout_ms) {
    s_forward_cmd     = 0.0f;
    s_turn_cmd        = 0.0f;
    s_has_ros_cmd     = false;
    s_left_cmd_rad_s  = 0.0f;
    s_right_cmd_rad_s = 0.0f;
  }
}

void ros_bridge_get_commands(float &forward_cmd, float &turn_cmd, bool &has_ros_cmd) {
  forward_cmd  = s_forward_cmd;
  turn_cmd     = s_turn_cmd;
  has_ros_cmd  = s_has_ros_cmd;
}

bool ros_bridge_reset_requested() {
  return s_reset_requested;
}

void ros_bridge_clear_reset_request() {
  s_reset_requested = false;
}

void ros_bridge_publish_feedback(const MotorFeedback &fb) {
  // Packet format: "F,<left_pos>,<left_vel>,<right_pos>,<right_vel>\n"
  // All values are in degrees / deg·s⁻¹ to match the ROS hardware
  // interface expectation.  The leading 'F' lets the host distinguish
  // feedback lines from echoed OK / error responses.
#if ROS_BRIDGE_DEMO_FEEDBACK
  MotorFeedback demo_fb = get_demo_feedback();
  Serial.print("F,");
  Serial.print(demo_fb.left_pos_deg,  3);
  Serial.print(",");
  Serial.print(demo_fb.left_vel_dps,  3);
  Serial.print(",");
  Serial.print(demo_fb.right_pos_deg, 3);
  Serial.print(",");
  Serial.println(demo_fb.right_vel_dps, 3);
#else
  Serial.print("F,");
  Serial.print(fb.left_pos_deg,  3);
  Serial.print(",");
  Serial.print(fb.left_vel_dps,  3);
  Serial.print(",");
  Serial.print(fb.right_pos_deg, 3);
  Serial.print(",");
  Serial.println(fb.right_vel_dps, 3);
#endif
}
