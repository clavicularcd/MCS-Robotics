package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

public class Turret {

    private final DcMotorEx turretMotor;

    // ── Encoder constants (still used for position reads & soft limits) ──────
    private static final double COUNTS_PER_MOTOR_REV = 288.0;
    private static final double MAX_RPM              = 125.0;
    private static final double MAX_TICKS_PER_SECOND = MAX_RPM / 60.0 * COUNTS_PER_MOTOR_REV;
    // MAX_TICKS_PER_SECOND = 125/60 * 288 = 600 ticks/sec

    // ── Gear ratio ────────────────────────────────────────────────────────────
    // Core Hex drives a 72T gear which meshes with 240T gear on the turret.
    // Turret revolutions per motor revolution = 72 / 240 = 0.3
    // To rotate turret 90°: 0.25 turret revs / 0.3 = 0.833 motor revs = 240 counts
    private static final double GEAR_RATIO        = 72.0 / 240.0;
    private static final int    COUNTS_PER_90_DEG = 240; // ±240 counts = ±90° turret rotation
    private static final int    SOFT_LIMIT_COUNTS = COUNTS_PER_90_DEG; // 240

    // ── PD constants for auto tracking ───────────────────────────────────────
    // Output is raw motor power (-1.0 to 1.0) via RUN_WITHOUT_ENCODER.
    // No hub PID — our PD controller has direct control of the motor.
    // At kP = 0.02: 15° error → power 0.3, 30° error → power 0.6 (MAX_POWER clamp)
    // Tune kP up if too slow to track, down if it oscillates.
    private final double kP             = 0.03;  // power per degree of bearing error
    private final double kD             = 0.001; // power per (degree/sec) of error rate
    private double angleTolerance = 1.0;   // degrees — within this, treat as aligned
    // 1.0° gives a ±3-4px dead band on HuskyLens — tight enough for shooting
    // accuracy, wide enough to not oscillate from sensor noise.

    private double lastError      = 0;
    private double lastBearing    = 0;     // stored every updateAuto() call for isAligned() check
    private boolean firstAutoLoop = true;  // prevents D-term spike on mode transition

    // Maximum auto-tracking power — prevents slamming into hard stops
    private static final double MAX_POWER = 0.6;

    private final ElapsedTime timer = new ElapsedTime();

    // ── Manual drive constants ────────────────────────────────────────────────
    private static final double DEADZONE          = 0.1;
    private static final double MANUAL_SPEED      = 0.3; // max manual power (scaled by stick)

    // ── Hold power ────────────────────────────────────────────────────────────
    // Power used as max speed when in RUN_TO_POSITION hold mode.
    // holdPosition() is the ONLY place that uses RUN_TO_POSITION + hub PID,
    // which is correct — we want the hub to actively resist being pushed.
    private static final double HOLD_POWER = 0.6;

    // ─────────────────────────────────────────────────────────────────────────

    public Turret(HardwareMap hwMap) {
        turretMotor = hwMap.get(DcMotorEx.class, "turretMotor");

        // Set direction first — changing direction after a mode switch can
        // cause a brief current spike that jerks the motor.
        turretMotor.setDirection(DcMotorEx.Direction.FORWARD);
        turretMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Zero the encoder at whatever position the turret is physically at.
        // Manually align the turret to face forward before each match.
        turretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turretMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    /**
     * MANUAL MODE — call every loop in TeleOp.
     * Pass gamepad2.right_stick_x directly.
     * Uses RUN_WITHOUT_ENCODER + setPower() — no hub PID.
     * Enforces ±90° software limits — turret stops and holds at boundary.
     */
    public void updateManual(double gamepadInput) {
        if (Math.abs(gamepadInput) < DEADZONE) gamepadInput = 0;
        gamepadInput = Range.clip(gamepadInput, -1.0, 1.0);

        // Block movement at software limits
        int currentPos = turretMotor.getCurrentPosition();
        if (currentPos >= SOFT_LIMIT_COUNTS  && gamepadInput > 0) gamepadInput = 0;
        if (currentPos <= -SOFT_LIMIT_COUNTS && gamepadInput < 0) gamepadInput = 0;

        if (gamepadInput == 0) {
            holdPosition();
            return;
        }

        // Switch out of RUN_TO_POSITION (set by holdPosition) before setPower
        ensureRunWithoutEncoder();

        double power = gamepadInput * MANUAL_SPEED;
        turretMotor.setPower(power);
    }

    /**
     * Call once when entering auto-tracking mode.
     * Resets PD state so the D-term doesn't spike from stale timer values.
     */
    public void resetAutoState() {
        lastError      = 0;
        lastBearing    = 0;
        firstAutoLoop  = true;
        timer.reset();
    }

    /**
     * AUTO TRACKING MODE — call every loop when using HuskyLens bearing.
     * Pass tag.bearingDeg from HuskyLensReader — turret will PD-track to zero bearing.
     * Uses RUN_WITHOUT_ENCODER + setPower() — our PD has direct motor control.
     * Output is raw power (-1.0 to 1.0), clamped to ±MAX_POWER.
     * Enforces ±90° software limits.
     * Returns commanded power for telemetry.
     */
    public double updateAuto(double bearingDeg) {
        double deltaTime = timer.seconds();
        timer.reset();

        double error = -bearingDeg; // goal is to center the tag (bearing = 0)
        double pTerm = error * kP;  // raw power

        // Suppress D-term on the first call after a mode switch to avoid
        // a huge spike from stale deltaTime or uninitialized lastError.
        double dTerm = 0;
        if (!firstAutoLoop && deltaTime > 0 && deltaTime < 1.0) {
            // Also ignore deltaTime > 1s as it means the loop stalled or
            // we just transitioned — stale data would produce garbage.
            dTerm = ((error - lastError) / deltaTime) * kD;
        }
        firstAutoLoop = false;

        double power;
        if (Math.abs(error) < angleTolerance) {
            power = 0;
        } else {
            power = Range.clip(pTerm + dTerm, -MAX_POWER, MAX_POWER);
        }

        // Enforce software limits
        int currentPos = turretMotor.getCurrentPosition();
        if (currentPos >= SOFT_LIMIT_COUNTS  && power > 0) power = 0;
        if (currentPos <= -SOFT_LIMIT_COUNTS && power < 0) power = 0;

        if (power == 0) {
            holdPosition();
        } else {
            // Switch out of RUN_TO_POSITION before setPower for direct control
            ensureRunWithoutEncoder();
            turretMotor.setPower(power);
        }

        lastError   = error;
        lastBearing = bearingDeg;
        return power;
    }

    // ── Shared helper: ensure motor is in direct-drive mode ─────────────────

    /**
     * Switches motor to RUN_WITHOUT_ENCODER if it isn't already.
     * In this mode, setPower() sends raw voltage to the motor — no hub PID.
     * Encoder still works for getCurrentPosition() reads.
     */
    private void ensureRunWithoutEncoder() {
        if (turretMotor.getMode() != DcMotor.RunMode.RUN_WITHOUT_ENCODER) {
            turretMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
    }

    /**
     * Holds the turret at its current position using RUN_TO_POSITION.
     * This is the ONLY method that uses the hub's internal PID — intentionally,
     * because we want active resistance to being pushed when holding still.
     * Called automatically when at a software limit or when input is zero.
     */
    private void holdPosition() {
        turretMotor.setTargetPosition(turretMotor.getCurrentPosition());
        turretMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        turretMotor.setPower(HOLD_POWER);
    }

    /**
     * Returns true if the turret is centred on the target within angleTolerance.
     * Use in TeleOp to detect when auto-align is complete and disengage.
     */
    public boolean isAligned() {
        return Math.abs(lastBearing) < angleTolerance;
    }

    /**
     * Returns the current turret angle in degrees.
     * Positive = right, negative = left, 0 = forward.
     */
    public double getAngleDeg() {
        return turretMotor.getCurrentPosition() / (double) COUNTS_PER_90_DEG * 90.0;
    }

    /**
     * Returns true if the turret is at or beyond either software limit.
     */
    public boolean isAtLimit() {
        int pos = turretMotor.getCurrentPosition();
        return pos >= SOFT_LIMIT_COUNTS || pos <= -SOFT_LIMIT_COUNTS;
    }

    /**
     * Returns the turret to the forward position (0°) using RUN_TO_POSITION.
     * Call once to trigger — motor holds position when done automatically.
     * Used when no navigation tag is visible during auto-align.
     */
    public void returnToCenter() {
        turretMotor.setTargetPosition(0);
        turretMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        turretMotor.setPower(HOLD_POWER);
    }

    /** Full stop — releases hold. */
    public void stop() {
        turretMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        turretMotor.setPower(0);
    }
}
