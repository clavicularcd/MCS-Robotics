package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

public class ShooterHood {

    private final CRServo        hoodServo;
    private final DcMotor        flywheel;

//    // ── Physical constants — fill in after calibration ───────────────────────
//    private static final double GOAL_HEIGHT_MM    = 984.5;  // TODO: measure
//    private static final double SHOOTER_HEIGHT_MM = ;  // TODO: measure
//    private static final double VERTICAL_RISE     = GOAL_HEIGHT_MM - SHOOTER_HEIGHT_MM;

    // ── Tunable factors ───────────────────────────────────────────────────────
    // TIME_FACTOR: seconds per degree of hood rotation. Tune until correct.
    private static final double TIME_FACTOR              = 0.2;
    private static final double MAX_HOOD_ANGLE           = 11.0;
    private static final double FLYWHEEL_SPINNING_PERIOD = 5.0;   //check this
    private static final double DEFAULT_HOOD_SERVO_POWER = 1.0;
    private static final double HOMING_SERVO_POWER       = 1.0;
    //private static final double DEFAULT_FLYWHEEL_POWER   = 0.6;  //shootPose1
    private static final double DEFAULT_FLYWHEEL_POWER = 0.65;  //shootPose2

    // ── Servo time cap ────────────────────────────────────────────────────────
    // 2.20 seconds is the full travel time of the hood servo.
    // Any time above this is silently clamped down to MAX_SERVO_TIME.
    private static final double MAX_SERVO_TIME          = MAX_HOOD_ANGLE * TIME_FACTOR;
    private static final double HOMING_TIMEOUT_SECONDS  = MAX_SERVO_TIME; // safety cutoff

//    // ── Calibration table — fill in from your calibration sheet ──────────────
//    // Distances in mm, corrections in degrees. Must be in ascending distance order.
//    private static final double[] CAL_DISTANCES   = { 500, 750, 1000, 1250, 1500, 1750, 2000 };
//    private static final double[] CAL_CORRECTIONS = { 0.0, 0.0,  0.0,  0.0,  0.0,  0.0,  0.0 };
//    // ↑ Replace 0.0 values with your measured corrections after calibration

    // ── State machine ─────────────────────────────────────────────────────────
    public enum LaunchState {
        IDLE,
        POWER_SERVO_AND_FLYWHEEL,
        STOP_SERVO,
        STOP_FLYWHEEL,
        RETRACT_SERVO          // homing back to zero after every shot
    }

    private LaunchState       currentState         = LaunchState.IDLE;
    private final ElapsedTime stopwatch            = new ElapsedTime();
    private double            timeToRotate         = 0;
    // servoPosition: current hood position in seconds of full-power travel
    // 0.0 = highest position, MAX_SERVO_TIME = lowest position
    private double            servoPosition        = 0.0;
    private double            retractStartPosition = 0.0;

    // ── Constructor ───────────────────────────────────────────────────────────

    public ShooterHood(HardwareMap hwMap) {
        hoodServo = hwMap.get(CRServo.class, "s1AsServo");
        flywheel  = hwMap.get(DcMotor.class, "flywheel");

        hoodServo.setDirection(CRServo.Direction.FORWARD);
        hoodServo.setPower(0);

        // Flywheel brakes immediately when power is set to zero
        flywheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        flywheel.setPower(0);
    }

    // ── Main update loop ──────────────────────────────────────────────────────

    /**
     * Call every loop in TeleOp / Auto to drive the state machine.
     */
    public void update() {
        switch (currentState) {

            case IDLE:
                // waiting for a triggerLaunch call
                break;

            case POWER_SERVO_AND_FLYWHEEL:
                // Start servo and flywheel simultaneously, then move to STOP_SERVO
                // so the stopwatch is checked on the next loop iteration
                retractStartPosition = servoPosition;
                stopwatch.reset();
                flywheel.setPower(DEFAULT_FLYWHEEL_POWER);
                hoodServo.setPower(DEFAULT_HOOD_SERVO_POWER);
                currentState = LaunchState.STOP_SERVO;
                break;

            case STOP_SERVO:
                // Wait until the hood has rotated the required angle
                // Position advances at DEFAULT_HOOD_SERVO_POWER units per second
                servoPosition = Math.min(
                        retractStartPosition + stopwatch.seconds() * DEFAULT_HOOD_SERVO_POWER,
                        MAX_SERVO_TIME);
                if (stopwatch.seconds() >= timeToRotate) {
                    hoodServo.setPower(0);
                    currentState = LaunchState.STOP_FLYWHEEL;
                }
                break;

            case STOP_FLYWHEEL:
                // Wait until ball has been launched, then brake the flywheel
                // NOTE: FLYWHEEL_SPINNING_PERIOD must be >> TIME_FACTOR * angleToRotate
                if (stopwatch.seconds() >= FLYWHEEL_SPINNING_PERIOD) {
                    flywheel.setPower(0); // ZeroPowerBehavior.BRAKE kicks in immediately
                    retractStartPosition = servoPosition;
                    currentState = LaunchState.RETRACT_SERVO;
                    stopwatch.reset();
                }
                break;

            case RETRACT_SERVO:
                hoodServo.setPower(-HOMING_SERVO_POWER);
                // Position retreats at HOMING_SERVO_POWER units per second
                servoPosition = Math.max(
                        retractStartPosition - stopwatch.seconds() * HOMING_SERVO_POWER,
                        0.0);
                if (servoPosition <= 0.0) {
                    // Position reached zero — we're home
                    hoodServo.setPower(0);
                    currentState = LaunchState.IDLE;
                } else if (stopwatch.seconds() > HOMING_TIMEOUT_SECONDS) {
                    // Safety: position never reached zero — stop anyway to avoid damage
                    hoodServo.setPower(0);
                    servoPosition = 0.0;
                    currentState = LaunchState.IDLE;
                }
                break;
        }
    }

    // ── Trigger methods ───────────────────────────────────────────────────────

    /**
     * Trigger a launch at a manually specified hood angle (degrees).
     * Angle is clamped to MAX_SERVO_TIME automatically.
     * Ignored if a launch is already in progress.
     */
    public void triggerLaunch(double angleDeg) {
        if (currentState == LaunchState.IDLE) {
            timeToRotate = clampTime(angleDeg * TIME_FACTOR);
            currentState = LaunchState.POWER_SERVO_AND_FLYWHEEL;
        }
    }

//    public void triggerLaunchForDistance(double distanceMm) {
//        if (currentState == LaunchState.IDLE) {
//            double geoAngle   = Math.toDegrees(Math.atan2(VERTICAL_RISE, distanceMm));
//            double correction = interpolateCorrection(distanceMm);
//            timeToRotate      = clampTime((geoAngle + correction) * TIME_FACTOR);
//            currentState      = LaunchState.POWER_SERVO_AND_FLYWHEEL;
//        }
//    }

    // ── Manual flywheel control ───────────────────────────────────────────────

    /**
     * Starts the flywheel independently of a launch sequence.
     * Used in TeleOp to spin up before a kick.
     * Safe to call even if flywheel is already running.
     */
    public void startFlywheel() {
        flywheel.setPower(DEFAULT_FLYWHEEL_POWER);
    }

    public void startAutoFlywheel(double power) { flywheel.setPower(power); }

    /**
     * Stops the flywheel immediately.
     * Only call this when not in the middle of a launch sequence.
     */
    public void stopFlywheel() {
        flywheel.setPower(0);
    }

    // ── Manual hood nudge ─────────────────────────────────────────────────────

    /**
     * Nudges the hood servo while a button is held.
     * Pass positive power to move up, negative to move down.
     * Ignored if a launch sequence is in progress.
     * Call with power = 0 when button is released to stop the servo.
     */
    public void nudge(double power) {
        if (currentState != LaunchState.IDLE) return;
        hoodServo.setPower(power);
    }

    // ── Status ────────────────────────────────────────────────────────────────

    /** True if any launch or retract sequence is currently running. */
    public boolean isBusy() {
        return currentState != LaunchState.IDLE;
    }

    public LaunchState getState() {
        return currentState;
    }

    /** Returns the current servo position in seconds of full-power travel
     *  (0.0 = highest, MAX_SERVO_TIME = lowest). */
    public double getServoPosition() {
        return servoPosition;
    }

    // ── Private helpers ───────────────────────────────────────────────────────

    /**
     * Clamps servo time to [0, MAX_SERVO_TIME].
     */
    private double clampTime(double timeSec) {
        return Math.max(0, Math.min(timeSec, MAX_SERVO_TIME));
    }

//    private double interpolateCorrection(double distanceMm) {
//        if (distanceMm <= CAL_DISTANCES[0])
//            return CAL_CORRECTIONS[0];
//
//        int last = CAL_DISTANCES.length - 1;
//        if (distanceMm >= CAL_DISTANCES[last])
//            return CAL_CORRECTIONS[last];
//
//        for (int i = 0; i < last; i++) {
//            if (distanceMm >= CAL_DISTANCES[i] && distanceMm < CAL_DISTANCES[i + 1]) {
//                double t = (distanceMm - CAL_DISTANCES[i])
//                        / (CAL_DISTANCES[i + 1] - CAL_DISTANCES[i]);
//                return CAL_CORRECTIONS[i] + t * (CAL_CORRECTIONS[i + 1] - CAL_CORRECTIONS[i]);
//            }
//        }
//        return 0.0;
//    }
}
