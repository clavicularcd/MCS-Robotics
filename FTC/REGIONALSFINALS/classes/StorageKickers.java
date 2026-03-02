package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

public class StorageKickers {

    private Servo storageServo1, storageServo2, storageServo3;
    private ElapsedTime stopwatch = new ElapsedTime();

    // ── Per-servo starting positions — tuned from field testing ──────────────
    private static final double STARTING_POSITION_1 = 0.21;
    private static final double STARTING_POSITION_2 = 0.08;
    private static final double STARTING_POSITION_3 = 0.3;

    // ── Kick amount and timings — tuned from field testing ───────────────────
    private static final double KICK_AMOUNT = 0.38; // rotateFactor from test
    private static final double DELAY_KICKER_STAYING_UP = 1.2;  // seconds
    private static final double DELAY_BETWEEN_KICKS = 1.2;  // seconds

    private enum SequenceState {
        IDLE,
        SERVO_WAIT_UP,
        SERVO_WAIT_DOWN
    }

    private SequenceState state = SequenceState.IDLE;

    // Which servo is currently active (1, 2, or 3). 0 = none.
    private int activeServo = 0;

    // Queue of servos to kick in order.
    private int[] queue = new int[0];
    private int queueIndex = 0;

    public StorageKickers(HardwareMap hwMap) {
        storageServo1 = hwMap.get(Servo.class, "storageServo1");
        storageServo2 = hwMap.get(Servo.class, "storageServo2");
        storageServo3 = hwMap.get(Servo.class, "storageServo3");

        // Servo 3 is physically reversed
        storageServo3.setDirection(Servo.Direction.REVERSE);

        storageServo1.setPosition(STARTING_POSITION_1);
        storageServo2.setPosition(STARTING_POSITION_2);
        storageServo3.setPosition(STARTING_POSITION_3);
    }

    /**
     * Call once per loop in TeleOp — drives the non-blocking state machine.
     */
    public void update() {
        switch (state) {

            case IDLE:
                if (queueIndex < queue.length) {
                    activeServo = queue[queueIndex];
                    getServo(activeServo).setPosition(getStartingPosition(activeServo) + KICK_AMOUNT);
                    stopwatch.reset();
                    state = SequenceState.SERVO_WAIT_UP;
                }
                break;

            case SERVO_WAIT_UP:
                if (stopwatch.seconds() >= DELAY_KICKER_STAYING_UP) {
                    getServo(activeServo).setPosition(getStartingPosition(activeServo));
                    stopwatch.reset();
                    state = SequenceState.SERVO_WAIT_DOWN;
                }
                break;

            case SERVO_WAIT_DOWN:
                if (stopwatch.seconds() >= DELAY_BETWEEN_KICKS) {
                    queueIndex++;
                    state = SequenceState.IDLE;
                }
                break;
        }
    }

    /**
     * Queue a single servo kick. servoNumber = 1, 2, or 3.
     * Ignored if already busy.
     */
    public void kickOne(int servoNumber) {
        if (state == SequenceState.IDLE) {
            queue = new int[]{servoNumber};
            queueIndex = 0;
        }
    }

//    /**
//     * Queue all three servos in sequence (1 → 2 → 3).
//     * Ignored if already busy.
//     */
//    public void kick_one_two_three() {
//        if (state == SequenceState.IDLE) {
//            queue = new int[]{1, 2, 3};
//            queueIndex = 0;
//        }
//    }
//    public void kick_two_one_three(){
//        if (state == SequenceState.IDLE) {
//            queue = new int[]{2, 1, 3};
//            queueIndex = 0;
//        }
//    }
//
//    public void kick_three_two_one(){
//        if (state == SequenceState.IDLE) {
//            queue = new int[]{3, 2, 1};
//            queueIndex = 0;
//        }
//    }


    /**
     * Queue a custom kick order produced by MotifResolver.
     * Pass the resolved int array e.g. kickInOrder(new int[]{ 2, 1, 3 })
     * Ignored if already busy.
     */
    public void kickInOrder(int[] order) {
        if (state == SequenceState.IDLE) {
            queue = order;
            queueIndex = 0;
        }
    }

    /**
     * Blocking version for use in Autonomous.
     * Kicks all three servos in sequence and waits for completion.
     * Do NOT call from TeleOp loop.
     */
    public void kickAllBlocking() {
        kickServoBlocking(storageServo1, STARTING_POSITION_1);
        kickServoBlocking(storageServo2, STARTING_POSITION_2);
        kickServoBlocking(storageServo3, STARTING_POSITION_3);
    }

    /**
     * True if a kick sequence is currently in progress.
     */
    public boolean isBusy() {
        return state != SequenceState.IDLE || queueIndex < queue.length;
    }

    // ── Private helpers ───────────────────────────────────────────────────────

    private Servo getServo(int number) {
        switch (number) {
            case 1:
                return storageServo1;
            case 2:
                return storageServo2;
            case 3:
                return storageServo3;
            default:
                return storageServo1; // safety fallback
        }
    }

    private double getStartingPosition(int number) {
        switch (number) {
            case 1:
                return STARTING_POSITION_1;
            case 2:
                return STARTING_POSITION_2;
            case 3:
                return STARTING_POSITION_3;
            default:
                return STARTING_POSITION_1; // safety fallback
        }
    }

    private void kickServoBlocking(Servo servo, double startingPosition) {
        ElapsedTime t = new ElapsedTime();

        servo.setPosition(startingPosition + KICK_AMOUNT);
        t.reset();
        while (t.seconds() < DELAY_KICKER_STAYING_UP) { /* wait */ }

        servo.setPosition(startingPosition);
        t.reset();
        while (t.seconds() < DELAY_BETWEEN_KICKS) { /* wait */ }
    }
}
