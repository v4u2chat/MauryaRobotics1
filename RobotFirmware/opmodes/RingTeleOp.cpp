#include "NRL.h"

// ============================================================
//  RingTeleOp — driver-controlled: grab the ring, push the base
// ============================================================
//
//  CONTROLS
//    Left stick  up/down   drive forward / back
//    Right stick left/right turn
//    Hold LB               SLOW mode (precision lining up on the ring / base)
//
//    D-pad DOWN            arm to PICKUP height (down at the ring)
//    D-pad UP              arm to CARRY height (up, ring clear of the floor)
//    D-pad LEFT / RIGHT    nudge the arm a few degrees (hold to keep moving)
//
//    A                     close claw  (grab ring)
//    X                     open claw   (release ring)
//    Y                     AUTO-GRAB: arm down -> close claw -> arm up
//
//  ---- TUNE THESE FIRST ----------------------------------------------------
//  The servo angles below are STARTING GUESSES. Your arm runs through a gear,
//  so the useful range may be small (e.g. 60..120) or reversed. Put the robot
//  on blocks, run this OpMode, and use D-pad LEFT/RIGHT while watching the
//  "arm" telemetry value on the controller. Write down the angle where the
//  claw is at ring height and where the ring clears the floor, then paste
//  them into ARM_PICKUP / ARM_CARRY. Do the same for CLAW_OPEN / CLAW_CLOSED.
// ---------------------------------------------------------------------------

// Arm (geared lift) — SERVO_1
constexpr float ARM_PICKUP   = 150.0f;  // claw at ring height on the floor
constexpr float ARM_CARRY    =  60.0f;  // ring lifted, safe to drive
constexpr float ARM_MIN      =  20.0f;  // never go past these: protects the gear
constexpr float ARM_MAX      = 170.0f;
constexpr float ARM_NUDGE    =   1.0f;  // degrees per tick while holding LEFT/RIGHT (100 ticks/s)
constexpr float ARM_RAMP_DPS = 120.0f;  // arm speed, deg/s. Lower = gentler on the gear.

// Claw — SERVO_2
constexpr float CLAW_OPEN    =  20.0f;
constexpr float CLAW_CLOSED  = 100.0f;  // firm grip on the ring, but don't stall the servo

// Drive
constexpr float SLOW_SCALE   = 0.40f;   // speed when LB is held
constexpr float FAST_SCALE   = 1.00f;

// ---------------------------------------------------------------------------
//  Hardware
// ---------------------------------------------------------------------------
static HexaDCMotor leftMotor {{ .dirPin = MOTOR_L_DIR, .pwmPin = MOTOR_L_PWM }};
static HexaDCMotor rightMotor{{ .dirPin = MOTOR_R_DIR, .pwmPin = MOTOR_R_PWM, .flipped = true }};
//                                                                  ^ if the robot spins when you push
//                                                                    the stick forward, set this to false
static HexaServo   arm       {{ .signalPin = SERVO_1, .startAngle = ARM_CARRY }};
static HexaServo   claw      {{ .signalPin = SERVO_2, .startAngle = CLAW_OPEN }};
static TankDrive   drive(leftMotor, rightMotor);

class RingTeleOp : public NRLOpMode {
public:
    void init() override {
        // Ramp rate must be set before begin() so even the first move is gentle.
        arm.setRampRate(ARM_RAMP_DPS);

        arm.begin();            // servos first, then motors (per kit docs)
        claw.begin();
        leftMotor.begin();
        rightMotor.begin();

        drive.setScale(FAST_SCALE);
        _clawClosed = false;
    }

    void loop() override {
        // ---------- Drive: always under the driver's control ----------
        bool slow = gamepad1.pressed(BTN_LB);
        drive.setScale(slow ? SLOW_SCALE : FAST_SCALE);
        drive.drive(gamepad1.leftY(), gamepad1.rightX());

        // ---------- Auto-grab macro (owns arm + claw while it runs) ----------
        if (gamepad1.justPressed(BTN_Y) && !isActionRunning()) {
            runAction(sequential({
                instant([this]{ arm.setPosition(ARM_PICKUP); }),
                sleep_ms(700),                                   // let the arm reach the floor
                instant([this]{ claw.setPosition(CLAW_CLOSED); _clawClosed = true; }),
                sleep_ms(400),                                   // let the claw bite
                instant([this]{ arm.setPosition(ARM_CARRY); }),
            }));
        }

        // ---------- Manual arm + claw: only when no macro is running ----------
        if (!isActionRunning()) {
            // Preset heights
            if (gamepad1.justPressed(BTN_DPAD_DOWN)) arm.setPosition(ARM_PICKUP);
            if (gamepad1.justPressed(BTN_DPAD_UP))   arm.setPosition(ARM_CARRY);

            // Fine nudge (hold). Clamped so the gear can't be driven past its stops.
            float nudge = 0.0f;
            if (gamepad1.pressed(BTN_DPAD_RIGHT)) nudge += ARM_NUDGE;
            if (gamepad1.pressed(BTN_DPAD_LEFT))  nudge -= ARM_NUDGE;
            if (nudge != 0.0f) {
                float target = arm.getPosition() + nudge;
                if (target < ARM_MIN) target = ARM_MIN;
                if (target > ARM_MAX) target = ARM_MAX;
                arm.setPosition(target);
            }

            // Claw
            if (gamepad1.justPressed(BTN_A)) { claw.setPosition(CLAW_CLOSED); _clawClosed = true;  }
            if (gamepad1.justPressed(BTN_X)) { claw.setPosition(CLAW_OPEN);   _clawClosed = false; }
        }

        // ---------- Status on the controller screen ----------
        telemetry.addData("arm",   arm.getPosition());
        telemetry.addData("claw",  _clawClosed ? "CLOSED" : "open");
        telemetry.addData("speed", slow ? "SLOW" : "fast");
        telemetry.addData("batt",  power.getBatteryVoltage());
    }

    void stop() override {
        cancelActions();
        drive.stop();
        arm.detach();
        claw.detach();
    }

private:
    bool _clawClosed = false;
};

REGISTER_OPMODE(RingTeleOp, "Ring TeleOp", TELEOP);
