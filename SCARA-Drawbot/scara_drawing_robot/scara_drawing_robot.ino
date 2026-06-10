// ============================================================
//  SCARA Drawing Robot
//  MCU:     ESP32-S3-Zero
//  Drivers: TMC2209 (UART + STEP/DIR)
//  Arm 1:   204mm shoulder  |  Arm 2: 200mm elbow
//  Gear ratio: 88:20 = 4.4:1 on both joints
//  NEMA 17 steppers, 200 steps/rev, 1/16 microstepping
//  Homing: limit switches on ARM1_LIMIT / ARM2_LIMIT pins
//  GCode via USB Serial: G0 G1 G28 G92 M3 M5 M114
//  Jog:  J1+N / J1-N   J2+N / J2-N  (degrees)
//  Cal:  SETCAL X Y
// ============================================================

#include <TMCStepper.h>
#include <math.h>

// ── Pin definitions ──────────────────────────────────────────
// Arm 1 — shoulder
#define EN_PIN_1      5
#define DIR_PIN_1     7
#define STEP_PIN_1   12
#define RX_PIN_1      2
#define TX_PIN_1      1
#define SERIAL_PORT_1 Serial1

// Arm 2 — elbow / crank
#define EN_PIN_2      6
#define DIR_PIN_2     8
#define STEP_PIN_2   13
#define RX_PIN_2      3
#define TX_PIN_2      4
#define SERIAL_PORT_2 Serial2

// Limit switches (INPUT_PULLUP — LOW when triggered)
#define ARM1_LIMIT   10
#define ARM2_LIMIT    9

// TMC2209 sense resistor
#define R_SENSE      0.11f

// ── Pen servo — ledc PWM (ESP32 core 3.x, 12-bit resolution) ─
#define SERVO_PIN      11
#define PWM_FREQ       50      // 50Hz standard servo
#define PWM_RES        12      // 12-bit: 0-4095
#define SERVO_MIN_US   500     // pulse at 0°
#define SERVO_MAX_US   2500    // pulse at 180°

const int PEN_UP_DEG   = 10;   // pen lifted
const int PEN_DOWN_DEG = 40;   // pen on paper

uint32_t angleToDuty(int angle) {
  long microSec   = map(angle, 0, 180, SERVO_MIN_US, SERVO_MAX_US);
  long totalPeriod = 1000000 / PWM_FREQ;
  return (uint32_t)map(microSec, 0, totalPeriod, 0, (1 << PWM_RES) - 1);
}

// ── TMC2209 driver objects ────────────────────────────────────
TMC2209Stepper driver1(&SERIAL_PORT_1, R_SENSE, 0b00);
TMC2209Stepper driver2(&SERIAL_PORT_2, R_SENSE, 0b00);

// ── Stepper configuration ────────────────────────────────────
// NEMA 17: 200 full steps/rev, gear ratio 4.4:1, 1/16 microstep
// Calibrated correction factor: 1.19
const float MOTOR_STEPS_PER_REV = 200.0f;
const float USTEPS               = 16.0f;
const float GEAR_RATIO           = 4.4f;
const float BASE_SPD = (MOTOR_STEPS_PER_REV * USTEPS * GEAR_RATIO) / 360.0f;
// Step factor 1.32 — corrects measured 88-92% scale error
// Measured: 40mm commanded → 35-37mm actual → avg 90% → factor = 1.19/0.90 = 1.32
const float STEPS_PER_DEG_1 = BASE_SPD * 1.32f;
const float STEPS_PER_DEG_2 = BASE_SPD * 1.32f;

// Motor current (mA)
const uint16_t RUN_CURRENT_MA  = 800;
const uint16_t HOLD_CURRENT_MA = 200;

// Step pulse half-period in microseconds (lower = faster, min ~150)
// Drawing moves: 800us, transit moves: 400us, homing: 2000us
volatile uint32_t stepInterval_us = 800;    // drawing speed
const uint32_t TRANSIT_INTERVAL   = 400;    // GOHOME / rapid moves
const uint32_t HOME_INTERVAL_FAST = 800;    // homing fast approach
const uint32_t HOME_INTERVAL_SLOW = 2500;   // homing slow re-approach

// ── Machine geometry ─────────────────────────────────────────
// Four-bar linkage SCARA — correct physical model
// J1 drives Arm1 ONLY (independent of crank)
// J2 drives Crank ONLY (independent of Arm1)
//
// arm1_angle  = ARM1_HOME - theta1
// crank_angle = CRANK_HOME - theta2   ← theta1 has NO effect
//
// Fitted to 338-point 5-degree grid remap
// Average error 3.55mm, 332/338 points under 10mm
const float ARM1_LEN   = 204.0884f;
const float ARM2_LEN   = 202.8832f;
const float ARM1_HOME  = -121.2814f;
const float CRANK_HOME =  -92.4086f;
const float CRANK_LEN  =   50.0f;
const float RED_ROD    =  200.0f;
const float PIVOT_SEP  =   50.0f;
const float BRACKET    =   30.0f;

// IK joint limits (degrees CW from home)
const float T1_MIN =   0.0f;
const float T1_MAX = 180.0f;
const float T2_MIN =  40.0f;   // below ~40° crank crosses arm1 toggle zone
const float T2_MAX = 180.0f;

// ── Canvas coordinate system ──────────────────────────────────
// GCode (0,0) = centre of drawing rectangle
// Drawing rect: robot X=-240..27, Y=-295..-123 (267x172mm)
// GCode (0,0) = FK at GOHOME (J1=50, J2=134) = (-100.3,-209.2)
const float CANVAS_ORIGIN_X = -100.3f;
const float CANVAS_ORIGIN_Y = -209.2f;

// Calibration offset — use G92 X Y to adjust after homing if needed
float CAL_OFFSET_X = 0.0f;
float CAL_OFFSET_Y = 0.0f;

// ── Motion parameters ─────────────────────────────────────────
float segmentLen_mm = 5.0f;

// ── State ─────────────────────────────────────────────────────
float theta1    = 0.0f;   // shoulder CW degrees from home
float theta2    = 0.0f;   // elbow    CW degrees from home
bool  penIsDown = false;
bool  motorsOn  = false;

String cmdBuffer = "";

// ============================================================
//  MOTOR ENABLE / DISABLE
// ============================================================
void enableMotors() {
  if (!motorsOn) {
    digitalWrite(EN_PIN_1, LOW);
    digitalWrite(EN_PIN_2, LOW);
    motorsOn = true;
  }
}

void disableMotors() {
  digitalWrite(EN_PIN_1, HIGH);
  digitalWrite(EN_PIN_2, HIGH);
  motorsOn = false;
}

// ============================================================
//  STEP HELPERS  — one step per call
// ============================================================
void stepMotor1(int dir) {
  // dir > 0 = CW = increasing theta1
  digitalWrite(DIR_PIN_1, dir > 0 ? HIGH : LOW);
  delayMicroseconds(2);
  digitalWrite(STEP_PIN_1, HIGH);
  delayMicroseconds(stepInterval_us);
  digitalWrite(STEP_PIN_1, LOW);
  delayMicroseconds(stepInterval_us);
}

void stepMotor2(int dir) {
  // dir > 0 = CW = increasing theta2
  digitalWrite(DIR_PIN_2, dir > 0 ? HIGH : LOW);
  delayMicroseconds(2);
  digitalWrite(STEP_PIN_2, HIGH);
  delayMicroseconds(stepInterval_us);
  digitalWrite(STEP_PIN_2, LOW);
  delayMicroseconds(stepInterval_us);
}

// ============================================================
//  FORWARD KINEMATICS — four-bar linkage, correct physical model
//  theta1 = shoulder motor CW degrees from home (Arm1 only)
//  theta2 = elbow motor CW degrees from home (Crank only)
//  They are INDEPENDENT — theta1 does not affect crank angle
// ============================================================
void forwardKinematics(float t1, float t2, float &x, float &y) {
  // Arm1 rotates with theta1
  float a1r = radians(ARM1_HOME - t1);
  float rbx  = ARM1_LEN * cosf(a1r);   // arm1 tip = right bracket top
  float rby  = ARM1_LEN * sinf(a1r);

  // Crank rotates with theta2 ONLY
  float crankr = radians(CRANK_HOME - t2);
  float ctx    = CRANK_LEN * cosf(crankr);
  float cty    = CRANK_LEN * sinf(crankr);

  // Four-bar analytical solution:
  // left_bracket_top = (rbx - PIVOT_SEP*cos(θ), rby - PIVOT_SEP*sin(θ))
  // Constraint: |lbt - crank_tip|² = RED_ROD²
  // → (A·cos θ + B·sin θ) = C  where A=rbx-ctx, B=rby-cty
  // → θ = φ + arccos(C/R)
  float Av = rbx - ctx;
  float Bv = rby - cty;
  float Rv = sqrtf(Av*Av + Bv*Bv);
  if (Rv < 1e-6f) { x = rbx + ARM2_LEN; y = rby; return; }

  float phi = atan2f(Bv, Av);
  float Cv  = (Av*Av + Bv*Bv + PIVOT_SEP*PIVOT_SEP - RED_ROD*RED_ROD)
              / (2.0f * PIVOT_SEP);
  float ratio = Cv / Rv;
  if (ratio > 1.0f) ratio = 1.0f;
  if (ratio < -1.0f) ratio = -1.0f;

  float theta = phi + acosf(ratio);   // branch +1

  float ct = cosf(theta), st = sinf(theta);

  // Right pivot on Arm2 tube (where Arm1 bracket connects)
  float rpx = rbx + BRACKET * st;
  float rpy = rby - BRACKET * ct;

  // Pen tip = right pivot + ARM2 along arm2 direction
  x = rpx + ARM2_LEN * ct;
  y = rpy + ARM2_LEN * st;
}

// ============================================================
//  INVERSE KINEMATICS — Newton-Raphson with numerical Jacobian
//  Much more robust than gradient descent for nonlinear four-bar
// ============================================================
bool inverseKinematics(float tx, float ty, float &t1, float &t2) {
  const float h = 1.0f;       // finite difference step (degrees) — larger for float stability
  const float tol = 1.0f;
  const int MAX_ITER = 200;

  for (int iter = 0; iter < MAX_ITER; iter++) {
    float px, py;
    forwardKinematics(t1, t2, px, py);
    float ex = tx - px, ey = ty - py;
    float err = sqrtf(ex*ex + ey*ey);
    if (err < tol) { return true; }

    // Numerical Jacobian with larger h for float stability
    float px1, py1, px2, py2;
    forwardKinematics(t1+h, t2,   px1, py1);
    forwardKinematics(t1,   t2+h, px2, py2);
    float J11=(px1-px)/h, J12=(px2-px)/h;
    float J21=(py1-py)/h, J22=(py2-py)/h;

    // Solve J * delta = e
    float det = J11*J22 - J12*J21;
    if (fabsf(det) < 0.01f) {
      t1 += 0.5f * ex; t2 += 0.5f * ey;
    } else {
      float d1 = ( J22*ex - J12*ey) / det;
      float d2 = (-J21*ex + J11*ey) / det;
      // Limit step size — allow larger steps
      float stepMax = 15.0f;
      float mag = sqrtf(d1*d1 + d2*d2);
      if (mag > stepMax) { d1 *= stepMax/mag; d2 *= stepMax/mag; }
      t1 += d1; t2 += d2;
    }

    // Clamp to valid range
    t1 = constrain(t1, T1_MIN, T1_MAX);
    t2 = constrain(t2, T2_MIN, T2_MAX);
  }

  float px, py;
  forwardKinematics(t1, t2, px, py);
  return sqrtf((px-tx)*(px-tx)+(py-ty)*(py-ty)) < 15.0f;
}

// ============================================================
//  IK WITH MULTI-START — tries several starting points
//  to avoid local minima in the nonlinear four-bar workspace
// ============================================================
bool inverseKinematicsMultiStart(float tx, float ty,
                                  float &t1_out, float &t2_out) {
  // Starting points scaled for step factor 1.32 — new model angles
  const float starts[][2] = {
    {50, 134}, {42, 135}, {58, 133},   // canvas centre and neighbours
    {35, 120}, {45, 145}, {55, 155},
    {25, 110}, {60, 130}, {40, 125},
    {30, 115}, {50, 150}, {20, 105},
    {t1_out, t2_out},
  };
  const int N_STARTS = 13;

  float bestErr = 1e9f;
  float bestT1 = t1_out, bestT2 = t2_out;

  for (int s = 0; s < N_STARTS; s++) {
    float t1 = starts[s][0], t2 = starts[s][1];
    inverseKinematics(tx, ty, t1, t2);
    float px, py;
    forwardKinematics(t1, t2, px, py);
    float err = sqrtf((px-tx)*(px-tx)+(py-ty)*(py-ty));
    if (err < bestErr) {
      bestErr = err; bestT1 = t1; bestT2 = t2;
    }
    if (bestErr < 1.0f) break;
  }

  t1_out = bestT1; t2_out = bestT2;
  return bestErr < 15.0f;
}

// ============================================================
//  MOVE MOTORS — Bresenham coordinated stepping
// ============================================================
void moveToAngles(float target1, float target2) {
  long steps1 = lroundf((target1 - theta1) * STEPS_PER_DEG_1);
  long steps2 = lroundf((target2 - theta2) * STEPS_PER_DEG_2);

  int  dir1  = steps1 >= 0 ? 1 : -1;
  int  dir2  = steps2 >= 0 ? 1 : -1;
  long abs1  = abs(steps1);
  long abs2  = abs(steps2);
  long major = max(abs1, abs2);
  long err1  = major / 2;
  long err2  = major / 2;

  for (long i = 0; i < major; i++) {
    err1 += abs1;
    if (err1 >= major) { err1 -= major; stepMotor1(dir1); }
    err2 += abs2;
    if (err2 >= major) { err2 -= major; stepMotor2(dir2); }
  }

  theta1 = target1;
  theta2 = target2;
}

// ============================================================
//  MOVE TO CARTESIAN — linear interpolation + IK per segment
// ============================================================
bool moveTo(float tx, float ty) {
  float cx, cy;
  forwardKinematics(theta1, theta2, cx, cy);
  float dx = tx - cx, dy = ty - cy;
  float dist = hypotf(dx, dy);
  if (dist < 0.01f) return true;

  bool inCanvasRegion = (theta1 > 10.0f && theta2 > 50.0f);

  // Workspace interpolation: solve IK at each point along the
  // straight line in robot XY space, tracking from previous solution
  int segs = max(1, (int)(dist / segmentLen_mm));

  // First: solve IK for full target to verify reachability
  float goalT1 = theta1, goalT2 = theta2;
  bool ok = (!inCanvasRegion)
    ? inverseKinematicsMultiStart(tx, ty, goalT1, goalT2)
    : inverseKinematics(tx, ty, goalT1, goalT2);

  if (!ok) {
    Serial.println("ERR: IK failed — point out of reach");
    return false;
  }

  Serial.print("IK: target("); Serial.print(tx,1);
  Serial.print(","); Serial.print(ty,1);
  Serial.print(") → t1="); Serial.print(goalT1,2);
  Serial.print(" t2="); Serial.println(goalT2,2);

  // Now interpolate in workspace, solving IK at each segment
  // starting from current position and tracking toward goal
  for (int i = 1; i <= segs; i++) {
    float fx = cx + dx * i / segs;
    float fy = cy + dy * i / segs;
    float nt1 = theta1, nt2 = theta2;
    // For last segment use the already-solved goal angles
    if (i == segs) {
      nt1 = goalT1; nt2 = goalT2;
    } else {
      inverseKinematics(fx, fy, nt1, nt2);
    }
    moveToAngles(nt1, nt2);
  }
  return true;
}

// ============================================================
//  PEN CONTROL — ledc PWM servo (ESP32 core 3.x)
// ============================================================
void penUp() {
  ledcWrite(SERVO_PIN, angleToDuty(PEN_UP_DEG));
  delay(200);
  penIsDown = false;
}

void penDown() {
  ledcWrite(SERVO_PIN, angleToDuty(PEN_DOWN_DEG));
  delay(200);
  penIsDown = true;
}

// ============================================================
//  HOMING — drive both CCW until limit switches close
// ============================================================
void home() {
  Serial.println("Homing...");
  penIsDown = false;
  enableMotors();

  uint32_t savedInterval = stepInterval_us;

  // Phase 1: fast approach
  stepInterval_us = HOME_INTERVAL_FAST;
  bool lim1 = false, lim2 = false;
  while (!lim1 || !lim2) {
    lim1 = (digitalRead(ARM1_LIMIT) == HIGH);
    lim2 = (digitalRead(ARM2_LIMIT) == HIGH);
    if (!lim1) stepMotor1(-1);
    if (!lim2) stepMotor2(-1);
  }

  // Phase 2: back off 5 degrees
  const long backoff = lroundf(5.0f * STEPS_PER_DEG_1);
  for (long i = 0; i < backoff; i++) {
    stepMotor1(1); stepMotor2(1);
  }

  // Phase 3: slow re-approach for repeatability
  stepInterval_us = HOME_INTERVAL_SLOW;
  lim1 = false; lim2 = false;
  while (!lim1 || !lim2) {
    lim1 = (digitalRead(ARM1_LIMIT) == HIGH);
    lim2 = (digitalRead(ARM2_LIMIT) == HIGH);
    if (!lim1) stepMotor1(-1);
    if (!lim2) stepMotor2(-1);
  }

  stepInterval_us = savedInterval;
  theta1 = 0.0f;
  theta2 = 0.0f;
  Serial.println("Home complete. theta1=0 theta2=0");
  Serial.println("OK");
}

// ============================================================
//  GCODE PARAMETER HELPER
// ============================================================
float gcodeParam(const String &line, char p) {
  int idx = line.indexOf(p);
  if (idx < 0) return NAN;
  return line.substring(idx + 1).toFloat();
}

// ============================================================
//  GCODE PARSER
// ============================================================
void processGCode(String line) {
  line.trim();
  line.toUpperCase();
  if (line.length() == 0 || line.startsWith(";")) return;

  int semi = line.indexOf(';');
  if (semi >= 0) line = line.substring(0, semi);
  line.trim();

  // ── GOHOME — move directly to canvas centre using known angles ─
  // Use this after G28 instead of G0 X0 Y0 to get into canvas region
  // Then use G0/G1 for drawing moves
  if (line.startsWith("GOHOME")) {
    // Canvas centre — J1=50, J2=134 → FK(-100,-209)
    const float HOME_T1 = 50.0f;
    const float HOME_T2 = 134.0f;
    enableMotors();
    uint32_t saved = stepInterval_us;
    stepInterval_us = TRANSIT_INTERVAL;
    moveToAngles(HOME_T1, HOME_T2);
    stepInterval_us = saved;
    float px, py;
    forwardKinematics(theta1, theta2, px, py);
    Serial.print("Canvas home: theta1="); Serial.print(theta1,1);
    Serial.print(" theta2="); Serial.print(theta2,1);
    Serial.print(" robot X="); Serial.print(px,1);
    Serial.print(" Y="); Serial.println(py,1);
    Serial.println("OK");
    return;
  }

  // ── G28  Home ─────────────────────────────────────────────
  if (line.startsWith("G28")) {
    home();
    return;
  }

  // ── G0 / G1  Move ─────────────────────────────────────────
  if (line.startsWith("G0") || line.startsWith("G1")) {
    float nx = gcodeParam(line, 'X');
    float ny = gcodeParam(line, 'Y');
    float cx, cy;
    forwardKinematics(theta1, theta2, cx, cy);
    float tx = isnan(nx) ? cx : nx + CANVAS_ORIGIN_X + CAL_OFFSET_X;
    float ty = isnan(ny) ? cy : ny + CANVAS_ORIGIN_Y + CAL_OFFSET_Y;
    enableMotors();
    bool isRapid = line.startsWith("G0");
    if (isRapid) stepInterval_us = TRANSIT_INTERVAL;
    bool ok = moveTo(tx, ty);
    if (isRapid) stepInterval_us = 800;
    if (ok) {
      Serial.print("OK X");
      Serial.print(tx - CANVAS_ORIGIN_X - CAL_OFFSET_X, 2);
      Serial.print(" Y");
      Serial.println(ty - CANVAS_ORIGIN_Y - CAL_OFFSET_Y, 2);
    }
    return;
  }

  // ── G92  Set position / calibration offset ─────────────────
  // G92 X0 Y0 tells the robot "current position IS canvas (0,0)"
  if (line.startsWith("G92")) {
    float nx = gcodeParam(line, 'X');
    float ny = gcodeParam(line, 'Y');
    float cx, cy;
    forwardKinematics(theta1, theta2, cx, cy);
    if (!isnan(nx)) CAL_OFFSET_X = cx - CANVAS_ORIGIN_X - nx;
    if (!isnan(ny)) CAL_OFFSET_Y = cy - CANVAS_ORIGIN_Y - ny;
    Serial.print("OK offset dX="); Serial.print(CAL_OFFSET_X, 2);
    Serial.print(" dY=");          Serial.println(CAL_OFFSET_Y, 2);
    return;
  }

  // ── M3  Pen down ───────────────────────────────────────────
  if (line.startsWith("M3")) { penDown(); Serial.println("OK"); return; }

  // ── M5  Pen up ─────────────────────────────────────────────
  if (line.startsWith("M5")) { penUp();   Serial.println("OK"); return; }

  // ── M114  Position report ──────────────────────────────────
  if (line.startsWith("M114")) {
    float cx, cy;
    forwardKinematics(theta1, theta2, cx, cy);
    Serial.print("POS: theta1=");  Serial.print(theta1, 2);
    Serial.print(" theta2=");      Serial.print(theta2, 2);
    Serial.print("  robot X=");    Serial.print(cx, 1);
    Serial.print(" Y=");           Serial.print(cy, 1);
    Serial.print("  canvas X=");   Serial.print(cx - CANVAS_ORIGIN_X - CAL_OFFSET_X, 1);
    Serial.print(" Y=");           Serial.println(cy - CANVAS_ORIGIN_Y - CAL_OFFSET_Y, 1);
    return;
  }

  // ── M17  Enable motors ─────────────────────────────────────
  if (line.startsWith("M17")) {
    enableMotors();
    Serial.println("OK motors enabled");
    return;
  }

  // ── M18  Disable motors ────────────────────────────────────
  if (line.startsWith("M18")) {
    disableMotors();
    Serial.println("OK motors disabled");
    return;
  }

  // ── M906 I<mA>  Set run current ────────────────────────────
  if (line.startsWith("M906")) {
    float ma = gcodeParam(line, 'I');
    if (!isnan(ma)) {
      driver1.rms_current((uint16_t)ma);
      driver2.rms_current((uint16_t)ma);
      Serial.print("OK current="); Serial.print((int)ma); Serial.println("mA");
    }
    return;
  }

  // ── SETCAL X Y  Measured-position calibration ──────────────
  // Move to G0 X0 Y0, physically measure pen position in robot
  // coordinates, then send: SETCAL X<measured_X> Y<measured_Y>
  if (line.startsWith("SETCAL")) {
    float ax = gcodeParam(line, 'X');
    float ay = gcodeParam(line, 'Y');
    if (!isnan(ax) && !isnan(ay)) {
      CAL_OFFSET_X = ax - CANVAS_ORIGIN_X;
      CAL_OFFSET_Y = ay - CANVAS_ORIGIN_Y;
      Serial.print("Cal offset: dX="); Serial.print(CAL_OFFSET_X, 2);
      Serial.print(" dY=");            Serial.println(CAL_OFFSET_Y, 2);
    }
    return;
  }

  // ── J1± / J2±  Jog N degrees ──────────────────────────────
  if (line.startsWith("J1") || line.startsWith("J2")) {
    bool isJ1    = (line.charAt(1) == '1');
    bool forward = (line.charAt(2) == '+');
    float deg    = line.substring(3).toFloat();
    long  steps  = lroundf(deg * (isJ1 ? STEPS_PER_DEG_1 : STEPS_PER_DEG_2));

    Serial.print("Jog "); Serial.print(isJ1 ? "J1" : "J2");
    Serial.print(forward ? "+" : "-"); Serial.print(deg, 1);
    Serial.print("  deg = "); Serial.print(steps); Serial.println(" steps");

    enableMotors();
    for (long i = 0; i < steps; i++) {
      if (isJ1) stepMotor1(forward ? 1 : -1);
      else      stepMotor2(forward ? 1 : -1);
    }

    if (isJ1) theta1 += forward ? deg : -deg;
    else      theta2 += forward ? deg : -deg;

    float cx, cy;
    forwardKinematics(theta1, theta2, cx, cy);
    Serial.print("JOG done: theta1="); Serial.print(theta1, 2);
    Serial.print(" theta2=");          Serial.print(theta2, 2);
    Serial.print("  robot X=");        Serial.print(cx, 1);
    Serial.print(" Y=");               Serial.println(cy, 1);
    return;
  }

  // ── SPEED N  Step interval µs ──────────────────────────────
  if (line.startsWith("SPEED")) {
    uint32_t us = (uint32_t)line.substring(5).toFloat();
    if (us >= 100 && us <= 5000) {
      stepInterval_us = us;
      Serial.print("OK step interval="); Serial.print(us); Serial.println("us");
    }
    return;
  }

  Serial.print("UNKNOWN: "); Serial.println(line);
}

// ============================================================
//  SETUP
// ============================================================
void setup() {
  Serial.begin(115200);
  delay(500);

  // Step / Dir / Enable pins
  for (int p : {EN_PIN_1, DIR_PIN_1, STEP_PIN_1,
                EN_PIN_2, DIR_PIN_2, STEP_PIN_2})
    pinMode(p, OUTPUT);
  digitalWrite(EN_PIN_1,   HIGH);   // disabled until needed
  digitalWrite(EN_PIN_2,   HIGH);
  digitalWrite(STEP_PIN_1, LOW);
  digitalWrite(STEP_PIN_2, LOW);

  // Pen servo — ledc 12-bit PWM (matches working servo test sketch)
  ledcAttach(SERVO_PIN, PWM_FREQ, PWM_RES);
  ledcWrite(SERVO_PIN, angleToDuty(PEN_UP_DEG));
  delay(500);

  // Limit switches
  pinMode(ARM1_LIMIT, INPUT_PULLUP);
  pinMode(ARM2_LIMIT, INPUT_PULLUP);

  // TMC2209 UART
  SERIAL_PORT_1.begin(115200, SERIAL_8N1, RX_PIN_1, TX_PIN_1);
  SERIAL_PORT_2.begin(115200, SERIAL_8N1, RX_PIN_2, TX_PIN_2);
  delay(200);

  // ── Driver 1 — shoulder ────────────────────────────────────
  driver1.begin();
  driver1.toff(5);
  driver1.blank_time(24);
  driver1.rms_current(RUN_CURRENT_MA);
  driver1.microsteps(16);
  driver1.en_spreadCycle(false);   // StealthChop (quiet)
  driver1.pwm_autoscale(true);
  driver1.ihold(HOLD_CURRENT_MA * 31 / RUN_CURRENT_MA);

  // ── Driver 2 — elbow ───────────────────────────────────────
  driver2.begin();
  driver2.toff(5);
  driver2.blank_time(24);
  driver2.rms_current(RUN_CURRENT_MA);
  driver2.microsteps(16);
  driver2.en_spreadCycle(false);
  driver2.pwm_autoscale(true);
  driver2.ihold(HOLD_CURRENT_MA * 31 / RUN_CURRENT_MA);

  Serial.println("SCARA Drawing Robot  —  ESP32-S3-Zero + TMC2209");
  Serial.println("Startup: G28 (home) then GOHOME (move to canvas centre)");
  Serial.println("Motion:  G0/G1 Xn Yn  M3  M5  M114");
  Serial.println("Jog:     J1+N  J1-N  J2+N  J2-N  (degrees)");
  Serial.println("Cal:     SETCAL X Y  |  G92 X Y");
  Serial.println("Tuning:  SPEED N (us)  |  M906 I<mA>  |  M17  M18");
  Serial.println("Ready. Send G28 then GOHOME to start.");
}

// ============================================================
//  MAIN LOOP
// ============================================================
void loop() {
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n' || c == '\r') {
      if (cmdBuffer.length() > 0) {
        processGCode(cmdBuffer);
        cmdBuffer = "";
      }
    } else {
      cmdBuffer += c;
    }
  }
}
