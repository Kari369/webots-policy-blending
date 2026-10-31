#include <math.h>
#include <stdbool.h>
#include <stdio.h>
#include <webots/compass.h>
#include <webots/gps.h>
#include <webots/keyboard.h>
#include <webots/motor.h>
#include <webots/robot.h>

#define TIME_STEP 16
#define MAX_SPEED 7.0       // rad/s
#define N_GOALS 3
#define GOAL_RADIUS 2.0     // m
#define TRIAL_TIMEOUT 90.0  // s
#define TURN_SAT 0.5        // rad: heading error at which the assist turns at full rate
#define LAMBDA 0.04         // how fast evidence updates the belief
#define EPSILON 0.002       // keeps every goal's belief above zero
#define ALPHA_MAX 0.6       // assisted mode: largest share of control given to the robot
#define ALPHA_FIXED 0.35    // no-inference mode: constant share of control given to the robot
#define COMMAND_DELAY_STEPS 25  // 25 steps * 16 ms = 0.4 s: operator's drive keys reach the robot late
#define N_OBSTACLES 3
#define OBSTACLE_RADIUS 0.5  // m: must match the cylinders in the world file
#define BODY_HALF_X 1.5      // m: robot footprint half length, from Moose.proto
#define BODY_HALF_Y 0.9      // m: robot footprint half width, including tires

enum { X, Y, Z };
enum { MODE_MANUAL, MODE_ASSISTED, MODE_NO_INFERENCE };
enum { IDLE, ARMED, RUNNING, DONE };
enum {
  K_UP = 1, K_DOWN = 2, K_LEFT = 4, K_RIGHT = 8,
  K_MANUAL = 16, K_ASSISTED = 32, K_NOINF = 64,
  K_G1 = 128, K_G2 = 256, K_G3 = 512
};

// goal positions in world X,Y (metres), ~12 m from the Moose start
static const double goals[N_GOALS][2] = {{-11.9, -1.1}, {-2.7, 3.2}, {6.5, -1.1}};
// obstacle centres in world X,Y (metres), halfway from the start to each goal
static const double obstacles[N_OBSTACLES][2] = {{-7.3, -4.95}, {-2.7, -2.8}, {1.9, -4.95}};
static const char *mode_names[] = {"manual", "assisted", "no_inference"};

static WbDeviceTag motors[8];
static WbDeviceTag gps;
static WbDeviceTag compass;

static double belief[N_GOALS];
static int mode = MODE_MANUAL;
static int state = IDLE;
static int intended = -1;
static int prev_mask = 0;
static double t0 = 0.0, path_len = 0.0, last_x = 0.0, last_y = 0.0;
static int ticks = 0;
static int collisions = 0;
static bool in_contact[N_OBSTACLES];
static char csv_path[512] = "trials.csv";
static double wrap_pi(double a) { return atan2(sin(a), cos(a)); }
static double clampd(double x, double lo, double hi) { return x < lo ? lo : (x > hi ? hi : x); }

static void set_wheels(double v, double w) {  // v forward, w turn left (both in [-1, 1])
  double left = MAX_SPEED * (v - w), right = MAX_SPEED * (v + w);
  double m = fmax(fabs(left), fabs(right));
  if (m > MAX_SPEED) {
    left *= MAX_SPEED / m;
    right *= MAX_SPEED / m;
  }
  for (int i = 0; i < 4; i++) {
    wb_motor_set_velocity(motors[i], left);
    wb_motor_set_velocity(motors[i + 4], right);
  }
}

static int read_keys() {
  int mask = 0, key;
  while ((key = wb_keyboard_get_key()) != -1) {
    switch (key & WB_KEYBOARD_KEY) {
      case WB_KEYBOARD_UP: mask |= K_UP; break;
      case WB_KEYBOARD_DOWN: mask |= K_DOWN; break;
      case WB_KEYBOARD_LEFT: mask |= K_LEFT; break;
      case WB_KEYBOARD_RIGHT: mask |= K_RIGHT; break;
      case 'M': mask |= K_MANUAL; break;
      case 'S': mask |= K_ASSISTED; break;
      case 'N': mask |= K_NOINF; break;
      case '1': mask |= K_G1; break;
      case '2': mask |= K_G2; break;
      case '3': mask |= K_G3; break;
    }
  }
  return mask;
}

static void reset_belief() {
  for (int g = 0; g < N_GOALS; g++)
    belief[g] = 1.0 / N_GOALS;
}

static int best_goal() {
  int b = 0;
  for (int g = 1; g < N_GOALS; g++)
    if (belief[g] > belief[b])
      b = g;
  return b;
}

static double angle_to_goal(int g, const double *pos) {
  return atan2(goals[g][1] - pos[Y], goals[g][0] - pos[X]);
}

// Step 1 of policy blending: infer which goal the operator is heading for.
// For each goal, the "expected" turn command is how far we must turn to face it.
// Goals whose expected turn matches the operator's actual turn input become more likely.
#define INFER_SAT 1.0   // rad: wider than TURN_SAT, so goals at different angles stay distinguishable
#define FWD_WEIGHT 0.5  // how much "driving toward it" counts as evidence

static void update_belief(double yaw, const double *pos, double hv, double hw) {
  double sum = 0.0;
  for (int g = 0; g < N_GOALS; g++) {
    double err = wrap_pi(angle_to_goal(g, pos) - yaw);
    double expected = clampd(err / INFER_SAT, -1.0, 1.0);
    double cost = (hw - expected) * (hw - expected) + FWD_WEIGHT * hv * (1.0 - cos(err));
    belief[g] *= exp(-LAMBDA * cost);
    sum += belief[g];
  }
  for (int g = 0; g < N_GOALS; g++)
    belief[g] = (1.0 - EPSILON) * (belief[g] / sum) + EPSILON / N_GOALS;
}
  

// True when the obstacle's circle overlaps the robot's rectangular footprint.
static bool touching_obstacle(int o, double yaw, const double *pos) {
  const double dx = obstacles[o][0] - pos[X], dy = obstacles[o][1] - pos[Y];
  const double fx = dx * cos(yaw) + dy * sin(yaw), fy = -dx * sin(yaw) + dy * cos(yaw);
  const double gap_x = fabs(fx) - clampd(fabs(fx), 0.0, BODY_HALF_X), gap_y = fabs(fy) - clampd(fabs(fy), 0.0, BODY_HALF_Y);
  return hypot(gap_x, gap_y) < OBSTACLE_RADIUS;
}

// Counts one collision each time the robot comes into contact with an obstacle.
static void update_collisions(double yaw, const double *pos, double elapsed) {
  for (int o = 0; o < N_OBSTACLES; o++) {
    const bool touching = touching_obstacle(o, yaw, pos);
    if (touching && !in_contact[o]) {
      collisions++;
      printf("COLLISION %d: obstacle %d at t=%.1fs\n", collisions, o + 1, elapsed);
    }
    in_contact[o] = touching;
  }
}

static void finish_trial(int reached, double elapsed) {
  int inferred = best_goal();
  bool success = (reached == intended);
  const char *label = reached < 0 ? "timeout" : (reached == 0 ? "G1" : (reached == 1 ? "G2" : "G3"));
  printf("TRIAL DONE: mode=%s intended=G%d reached=%s success=%d time=%.1fs path=%.1fm collisions=%d inferred=G%d\n",
         mode_names[mode], intended + 1, label, success, elapsed, path_len, collisions, inferred + 1);
  FILE *f = fopen(csv_path, "a");
  if (f) {
    fseek(f, 0, SEEK_END);
    if (ftell(f) == 0)
      fprintf(f, "mode,intended_goal,reached_goal,success,time_s,path_m,collisions,inferred_goal,inference_correct\n");
    fprintf(f, "%s,G%d,%s,%d,%.2f,%.2f,%d,G%d,%d\n", mode_names[mode], intended + 1, label, success, elapsed, path_len,
            collisions, inferred + 1, inferred == intended);
    fclose(f);
    printf("Saved to %s\n", csv_path);
  } else {
    printf("WARNING: could not open %s. Copy the TRIAL DONE line above by hand.\n", csv_path);
  }
  printf("Press the rewind button (next to Play) to reset for the next trial.\n");
  state = DONE;
  set_wheels(0.0, 0.0);
}

int main(int argc, char *argv[]) {
  wb_robot_init();

  printf("SHARED CONTROL. Click the 3D window first.\n");
  printf("Arrow keys: drive.  M = manual, S = assisted, N = no-inference assist\n");
  printf("1/2/3 = declare which goal you intend (arms a logged trial; timer starts on first arrow key)\n\n");

  wb_robot_step(1000);

    snprintf(csv_path, sizeof(csv_path), "%s/trials.csv", wb_robot_get_project_path());
  printf("Trial results will be saved to: %s\n", csv_path);

  const char *names[8] = {"left motor 1",  "left motor 2",  "left motor 3",  "left motor 4",
                          "right motor 1", "right motor 2", "right motor 3", "right motor 4"};
  for (int i = 0; i < 8; i++) {
    motors[i] = wb_robot_get_device(names[i]);
    if (motors[i] == 0) {
      printf("ERROR: device '%s' not found on this robot\n", names[i]);
      wb_robot_cleanup();
      return 1;
    }
    wb_motor_set_position(motors[i], INFINITY);
  }
  gps = wb_robot_get_device("gps");
  compass = wb_robot_get_device("compass");
  if (gps == 0 || compass == 0) {
    printf("ERROR: gps or compass not found on this robot\n");
    wb_robot_cleanup();
    return 1;
  }
  wb_gps_enable(gps, TIME_STEP);
  wb_compass_enable(compass, TIME_STEP);
  wb_keyboard_enable(TIME_STEP);

  reset_belief();
  set_wheels(0.0, 0.0);

  while (wb_robot_step(TIME_STEP) != -1) {
    const double *pos = wb_gps_get_values(gps);
    const double *north = wb_compass_get_values(compass);
    const double yaw = atan2(north[0], north[1]);  // same heading convention as the demo controller

    const int mask = read_keys();
    const int pressed = mask & ~prev_mask;  // keys that went down this step
    prev_mask = mask;

       static double last_cmd = -1.0;
    if ((state == IDLE || state == ARMED) && pressed && wb_robot_get_time() - last_cmd > 0.5) {
      if (pressed & K_MANUAL) { mode = MODE_MANUAL; printf("mode: manual\n"); }
      if (pressed & K_ASSISTED) { mode = MODE_ASSISTED; printf("mode: assisted\n"); }
      if (pressed & K_NOINF) { mode = MODE_NO_INFERENCE; printf("mode: no_inference\n"); }
      if (pressed & (K_G1 | K_G2 | K_G3)) {
        intended = (pressed & K_G1) ? 0 : ((pressed & K_G2) ? 1 : 2);
        reset_belief();
        state = ARMED;
        printf("Trial armed: mode=%s, intended goal G%d. Start driving.\n", mode_names[mode], intended + 1);
      }
      if (pressed & (K_MANUAL | K_ASSISTED | K_NOINF | K_G1 | K_G2 | K_G3))
        last_cmd = wb_robot_get_time();
    }

    if (state == DONE) {
      set_wheels(0.0, 0.0);
      continue;
    }

    // Experiment 2: simulate a slow teleoperation link by delaying the drive keys (all modes).
    static int delay_buf[COMMAND_DELAY_STEPS];
    static int delay_idx = 0;
    const int drive_mask = delay_buf[delay_idx];
    delay_buf[delay_idx] = mask;
    delay_idx = (delay_idx + 1) % COMMAND_DELAY_STEPS;

    const double hv = ((drive_mask & K_UP) ? 1.0 : 0.0) - ((drive_mask & K_DOWN) ? 1.0 : 0.0);
    const double hw = ((drive_mask & K_LEFT) ? 1.0 : 0.0) - ((drive_mask & K_RIGHT) ? 1.0 : 0.0);
    const bool active = (hv != 0.0 || hw != 0.0);

    if (state == ARMED && active) {
      state = RUNNING;
      t0 = wb_robot_get_time();
      path_len = 0.0;
      last_x = pos[X];
      last_y = pos[Y];
    }

    // inference runs in every mode (so inference accuracy can also be logged for manual runs)
    if (active && hv >= 0.0)
      update_belief(yaw, pos, hv, hw);

    // Step 2: blend operator command with goal-directed assistance
    double v = hv, w = hw;
    if (active && hv >= 0.0 && mode != MODE_MANUAL) {
      int target;
      double alpha;
      if (mode == MODE_ASSISTED) {
        target = best_goal();
        double conf = (belief[target] - 1.0 / N_GOALS) / (1.0 - 1.0 / N_GOALS);
        alpha = ALPHA_MAX * clampd(conf, 0.0, 1.0);
      } else {  // no inference: constant assist toward whichever goal is closest to straight ahead
        target = 0;
        double best = fabs(wrap_pi(angle_to_goal(0, pos) - yaw));
        for (int g = 1; g < N_GOALS; g++) {
          double e = fabs(wrap_pi(angle_to_goal(g, pos) - yaw));
          if (e < best) { best = e; target = g; }
        }
        alpha = ALPHA_FIXED;
      }
      const double err = wrap_pi(angle_to_goal(target, pos) - yaw);
      const double v_a = fmax(0.0, cos(err));
      const double w_a = clampd(err / TURN_SAT, -1.0, 1.0);
      v = (1.0 - alpha) * hv + alpha * v_a;
      w = (1.0 - alpha) * hw + alpha * w_a;
    }
    set_wheels(v, w);

    if (state == RUNNING) {
      path_len += hypot(pos[X] - last_x, pos[Y] - last_y);
      last_x = pos[X];
      last_y = pos[Y];
      const double elapsed = wb_robot_get_time() - t0;
      update_collisions(yaw, pos, elapsed);

            if (++ticks % 62 == 0) {
        printf("t=%.0fs belief G1=%.2f G2=%.2f G3=%.2f | dist G1=%.1f G2=%.1f G3=%.1f | hv=%.0f hw=%.0f\n", elapsed, belief[0],
               belief[1], belief[2], hypot(goals[0][0] - pos[X], goals[0][1] - pos[Y]),
               hypot(goals[1][0] - pos[X], goals[1][1] - pos[Y]), hypot(goals[2][0] - pos[X], goals[2][1] - pos[Y]), hv, hw);
      }

      int reached = -2;
      for (int g = 0; g < N_GOALS; g++)
        if (hypot(goals[g][0] - pos[X], goals[g][1] - pos[Y]) < GOAL_RADIUS)
          reached = g;
      if (reached >= 0)
        finish_trial(reached, elapsed);
      else if (elapsed > TRIAL_TIMEOUT)
        finish_trial(-1, elapsed);
    }
  }

  wb_robot_cleanup();
  return 0;
}