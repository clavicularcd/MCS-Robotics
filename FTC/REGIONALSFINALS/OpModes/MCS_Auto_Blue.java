package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * MCS Autonomous — Blue Alliance
 *
 * Starting position: (57, 9, 110°) on the back launch line.
 * Preloaded with 3 balls.
 *
 * Sequence:
 *   1. Move hood to AUTO_HOOD_ANGLE
 *   2. For each servo (1, 2, 3):
 *      a. Prespin flywheel for AUTO_PRESPIN_SECONDS
 *      b. Kick servo, wait for completion
 *      c. Stop flywheel
 *   3. Drive forward off the launch line for ranking points
 */
@Autonomous(name = "MCS Auto — Blue", group = "MCS")
public class MCS_Auto_Blue extends LinearOpMode {

    // ── Tune these ──────────────────────────────────────────────────────────
    private static final double AUTO_PRESPIN_SECONDS = 4.0;    // flywheel spool time before each kick
    private static final double AUTO_HOOD_ANGLE = 11.0;         // hood angle in degrees (0 = flat, no movement)
    private static final double DRIVE_FORWARD_SECONDS = 1.0;   // time to drive forward off the line
    private static final float  DRIVE_POWER = 0.5f;            // forward drive power

    private static final double AUTO_FLYWHEEL_POWER = 0.678;

    @Override
    public void runOpMode() {
        Drivetrain drivetrain = new Drivetrain(hardwareMap);
        ShooterHood hood = new ShooterHood(hardwareMap);
        StorageKickers kickers = new StorageKickers(hardwareMap);

        ElapsedTime timer = new ElapsedTime();

        telemetry.addData("Alliance", "BLUE");
        telemetry.addData("Start Pose", "(57, 9, 110°)");
        telemetry.addData("Hood angle", AUTO_HOOD_ANGLE);
        telemetry.addData("Prespin time", AUTO_PRESPIN_SECONDS);
        telemetry.addData("Status", "Ready");
        telemetry.update();

        waitForStart();

        // ── Set hood angle once for all shots ───────────────────────────
        if (AUTO_HOOD_ANGLE > 0 && opModeIsActive()) {
            double hoodMoveTime = AUTO_HOOD_ANGLE * 0.2; // 0.2s per degree (TIME_FACTOR)
            timer.reset();
            while (opModeIsActive() && timer.seconds() < hoodMoveTime) {
                hood.nudge(1.0);
            }
            hood.nudge(0);
        }

        // ── Kick all 3 servos in sequence with prespin ──────────────────
        for (int servo = 1; servo <= 3; servo++) {
            if (!opModeIsActive()) return;

            // Prespin flywheel
            telemetry.addData("Action", "Prespin for servo " + servo);
            telemetry.update();
            hood.startAutoFlywheel(AUTO_FLYWHEEL_POWER);
            timer.reset();
            while (opModeIsActive() && timer.seconds() < AUTO_PRESPIN_SECONDS) {
                // wait for prespin
            }

            if (!opModeIsActive()) return;

            // Kick
            telemetry.addData("Action", "Kicking servo " + servo);
            telemetry.update();
            kickers.kickOne(servo);
            while (opModeIsActive() && kickers.isBusy()) {
                kickers.update();
            }

            // Stop flywheel between kicks
            hood.stopFlywheel();
        }

        // ── Retract hood if it was moved ────────────────────────────────
        if (AUTO_HOOD_ANGLE > 0 && opModeIsActive()) {
            double hoodMoveTime = AUTO_HOOD_ANGLE * 0.2;
            timer.reset();
            while (opModeIsActive() && timer.seconds() < hoodMoveTime) {
                hood.nudge(-1.0);
            }
            hood.nudge(0);
        }

        // ── Drive forward off the launch line ───────────────────────────
        if (!opModeIsActive()) return;

        telemetry.addData("Action", "Driving forward");
        telemetry.update();
        timer.reset();
        while (opModeIsActive() && timer.seconds() < DRIVE_FORWARD_SECONDS) {
            drivetrain.drive(DRIVE_POWER, 0, 0, false);
            telemetry.addData("Driving", timer.seconds());
            telemetry.update();
        }
        drivetrain.stop();

        telemetry.addData("Action", "Done!");
        telemetry.update();
    }
}
