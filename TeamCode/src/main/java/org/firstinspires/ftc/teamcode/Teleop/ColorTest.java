package org.firstinspires.ftc.teamcode.Teleop;

import android.graphics.Color;

import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.PoseStorage;

import java.util.ArrayDeque;

@TeleOp
public class ColorTest extends LinearOpMode {

    // =========================
    // Hardware (class fields)
    // =========================
    private DcMotor frontLeft, frontRight, backLeft, backRight;
    private DcMotorEx leftLauncher, rightLauncher;
    private DcMotor leftBeltMotor, rightBeltMotor;

    private CRServo leftIntakeLeftServo, leftIntakeRightServo;
    private CRServo rightIntakeLeftServo, rightIntakeRightServo;
    private Servo rightIntakeTopServo, leftIntakeTopServo;

    private HuskyLens camera;

    private MecanumDrive drive;

    // =========================
    // State variables
    // =========================
    private int launcherDirection = 1; // 1 = normal, -1 = reversed

    // Right top intake positions
    private static final double RIGHT_UP_POS     = 0.51;
    private static final double RIGHT_DOWN_POS   = 0.472;
    private static final double RIGHT_MIDDLE_POS = 0.485;

    // Left top intake positions
    private static final double LEFT_UP_POS     = 0.475;
    private static final double LEFT_DOWN_POS   = 0.51;
    private static final double LEFT_MIDDLE_POS = 0.5;

    // Height thresholds (pixels) - tune these with telemetry
    private static final double H1 = 30;
    private static final double H2 = 34;
    private static final double H3 = 38;
    private static final double H4 = 42;

    // Matching powers for each zone (far -> near)
    private static final double V_FAR  = 2100;
    private static final double V_MID1 = 2000;
    private static final double V_MID2 = 1900;
    private static final double V_MID3 = 1800;
    private static final double V_NEAR = 1700;

    private double noTagVelocity = 1725;
    private static final double NO_TAG_STEP = 50;

    private boolean lastDpadUp = false;
    private boolean lastDpadDown = false;

    private static final double INTAKE_DEADBAND = 0.05;

    final int TargetTagID = 4;

    // ============================================================
    // COLOR SENSORS + LIGHTS (added)
    // ============================================================
    private NormalizedColorSensor leftColor, rightColor;
    private DistanceSensor leftDistance, rightDistance;
    private Servo LeftLight, RightLight;

    // Distance gate for reading color
    private static final double COLOR_CAP_CM = 3.5;

    // Tuned hue bands (your values)
    private static final float GREEN_MIN = 100f, GREEN_MAX = 180f;
    private static final float PURPLE_MIN = 200f, PURPLE_MAX = 280f;

    // Light servo positions
    private static final double GREEN_POS   = 0.50;
    private static final double PURPLE_POS  = 0.72;
    private static final double YELLOW_POS  = 0.385; // separator + pre/post flash
    private static final double UNKNOWN_POS = 1.00;

    // Log limits
    private static final int MAX_LOG = 3;
    private static final int STABLE_LOOPS_REQUIRED = 2;
    private static final int BREAK_LOOPS_REQUIRED  = 6;
    private static final int LAUNCH_GAP_LOOPS_REQUIRED = 2;

    // Flash timing: yellow 0.25s, colors 0.50s
    private static final double YELLOW_SEC = 0.25;
    private static final double COLOR_SEC  = 0.50;

    private final ArrayDeque<String> leftLog = new ArrayDeque<>(MAX_LOG);
    private final ArrayDeque<String> rightLog = new ArrayDeque<>(MAX_LOG);

    private float[] leftHsv = new float[3];
    private float[] rightHsv = new float[3];

    // Intake event state (left)
    private String leftLastSeen = "NONE";
    private int leftStableCount = 0;
    private int leftBreakCount = BREAK_LOOPS_REQUIRED;
    private boolean leftLoggedThisEvent = false;

    // Intake event state (right)
    private String rightLastSeen = "NONE";
    private int rightStableCount = 0;
    private int rightBreakCount = BREAK_LOOPS_REQUIRED;
    private boolean rightLoggedThisEvent = false;

    // Launch removal state (left/right)
    private boolean leftWasSeeingLaunchColor = false;
    private int leftLaunchGapCount = 0;

    private boolean rightWasSeeingLaunchColor = false;
    private int rightLaunchGapCount = 0;

    // Flash state
    private boolean lastX = false;
    private boolean leftFlashing = false;
    private double leftFlashStartSec = 0.0;

    private boolean rightFlashing = false;
    private double rightFlashStartSec = 0.0;

    // ============================================================

    @Override
    public void runOpMode() throws InterruptedException {
        initHardware();
        initDriveDirectionsAndBrakes();
        initColorSensorsAndLights(); // <-- added

        telemetry.addLine("Initialized. Waiting for start...");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            updateLocalization();

            handleDrive();
            handleBelts();
            handleIntakes();

            TopLeftIntake();
            TopRightIntake();

            // --- Color sensors + lights update (added) ---
            updateColorSensorsAndLights();

            double tagHeight = readTagHeight();      // <-- read ONCE
            handleLauncherPower(tagHeight);          // <-- use it
            updateTelemetry(tagHeight);              // <-- show it

            idle();
        }
    }

    // =========================
    // Initialization
    // =========================
    private void initHardware() {
        // Drivetrain motors
        frontLeft  = hardwareMap.get(DcMotor.class, "leftFront");
        frontRight = hardwareMap.get(DcMotor.class, "rightFront");
        backLeft   = hardwareMap.get(DcMotor.class, "leftBack");
        backRight  = hardwareMap.get(DcMotor.class, "rightBack");

        // Launchers
        leftLauncher  = hardwareMap.get(DcMotorEx.class, "LeftLauncher");
        rightLauncher = hardwareMap.get(DcMotorEx.class, "RightLauncher");

        leftLauncher.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightLauncher.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftLauncher.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightLauncher.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        leftLauncher.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        rightLauncher.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        // Left intake + belt
        leftIntakeLeftServo  = hardwareMap.get(CRServo.class, "LeftIntakeLeftServo");
        leftIntakeRightServo = hardwareMap.get(CRServo.class, "LeftIntakeRightServo");
        leftIntakeTopServo   = hardwareMap.get(Servo.class, "LeftIntakeTopServo");
        leftBeltMotor        = hardwareMap.get(DcMotor.class, "leftBeltMotor");

        // Right intake + belt
        rightIntakeLeftServo  = hardwareMap.get(CRServo.class, "RightIntakeLeftServo");
        rightIntakeRightServo = hardwareMap.get(CRServo.class, "RightIntakeRightServo");
        rightIntakeTopServo   = hardwareMap.get(Servo.class, "RightIntakeTopServo");
        rightBeltMotor        = hardwareMap.get(DcMotor.class, "rightBeltMotor");

        rightBeltMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        camera = hardwareMap.get(HuskyLens.class, "huskylens");
        camera.selectAlgorithm(HuskyLens.Algorithm.TAG_RECOGNITION);

        rightIntakeTopServo.setPosition(RIGHT_MIDDLE_POS);
        leftIntakeTopServo.setPosition(LEFT_MIDDLE_POS);

        // RoadRunner / drive wrapper
        drive = new MecanumDrive(hardwareMap, PoseStorage.currentPose);
    }

    // -------------------------
    // Color sensors + lights init (added)
    // -------------------------
    private void initColorSensorsAndLights() {
        // Names you said you have in config:
        leftColor     = hardwareMap.get(NormalizedColorSensor.class, "LeftColorSensor");
        leftDistance  = hardwareMap.get(DistanceSensor.class, "LeftColorSensor");
        LeftLight     = hardwareMap.get(Servo.class, "LeftLight");

        rightColor    = hardwareMap.get(NormalizedColorSensor.class, "RightColorSensor");
        rightDistance = hardwareMap.get(DistanceSensor.class, "RightColorSensor");
        RightLight    = hardwareMap.get(Servo.class, "RightLight");

        // If you want sensor LEDs on (usually helps):
        // leftColor.enableLight(true);
        // rightColor.enableLight(true);

        // Default to unknown
        LeftLight.setPosition(UNKNOWN_POS);
        RightLight.setPosition(UNKNOWN_POS);
    }

    private void updateTopIntakeServo(Servo servo,
                                      double intakeStickY,
                                      boolean downButton,
                                      double upPos,
                                      double downPos,
                                      double midPos) {

        boolean intakeRunning = Math.abs(intakeStickY) > INTAKE_DEADBAND;

        if (downButton) {
            servo.setPosition(downPos);     // button ALWAYS wins
        } else if (intakeRunning) {
            servo.setPosition(upPos);       // intake moving -> up
        } else {
            servo.setPosition(midPos);      // idle -> middle
        }
    }

    private void TopRightIntake() {
        updateTopIntakeServo(
                rightIntakeTopServo,
                gamepad2.right_stick_y,
                gamepad2.b,
                RIGHT_UP_POS,
                RIGHT_DOWN_POS,
                RIGHT_MIDDLE_POS
        );
    }

    private void TopLeftIntake() {
        updateTopIntakeServo(
                leftIntakeTopServo,
                gamepad2.left_stick_y,
                gamepad2.a,
                LEFT_UP_POS,
                LEFT_DOWN_POS,
                LEFT_MIDDLE_POS
        );
    }

    private double pickVelocityFromTagHeight(double tagHeight) {
        if (tagHeight < 0) return noTagVelocity; // no tag

        if (tagHeight < H1) return V_FAR;
        if (tagHeight < H2) return V_MID1;
        if (tagHeight < H3) return V_MID2;
        if (tagHeight < H4) return V_MID3;
        return V_NEAR;
    }

    private double readTagHeight() {
        double height = -1.0; // -1 = no tag found

        HuskyLens.Block[] blocks = camera.blocks();
        for (HuskyLens.Block b : blocks) {
            if (b.id == TargetTagID) {
                height = b.height;
                break;
            }
        }
        return height;
    }

    private void initDriveDirectionsAndBrakes() {
        frontLeft.setDirection(DcMotor.Direction.FORWARD);
        backLeft.setDirection(DcMotor.Direction.FORWARD);
        frontRight.setDirection(DcMotor.Direction.REVERSE);
        backRight.setDirection(DcMotor.Direction.REVERSE);

        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    // =========================
    // Loop helpers
    // =========================
    private void updateLocalization() {
        drive.updatePoseEstimate();
    }

    private void handleDrive() {
        double y  = -gamepad1.left_stick_y;
        double x  =  gamepad1.left_stick_x;
        double rx =  gamepad1.right_stick_x;

        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1.0);
        double fl = (y + x + rx) / denominator;
        double bl = (y - x + rx) / denominator;
        double fr = (y - x - rx) / denominator;
        double br = (y + x - rx) / denominator;

        double drivePower = getDrivePowerModifier();

        frontLeft.setPower(fl * drivePower);
        backLeft.setPower(bl * drivePower);
        frontRight.setPower(fr * drivePower);
        backRight.setPower(br * drivePower);
    }

    private double getDrivePowerModifier() {
        if (gamepad1.left_bumper) return 0.5;
        return 1.0;
    }

    private void handleBelts() {
        double leftBeltPower;
        if (gamepad2.left_bumper) {
            leftBeltPower = 1.0; // launch feed
        } else if (gamepad2.left_stick_y < 0) {
            leftBeltPower = -1.0; // intake feed
        } else {
            leftBeltPower = 0.0;
        }

        double rightBeltPower;
        if (gamepad2.right_bumper) {
            rightBeltPower = 1.0; // launch feed
        } else if (gamepad2.right_stick_y < 0) {
            rightBeltPower = -1.0; // intake feed
        } else {
            rightBeltPower = 0.0;
        }

        leftBeltMotor.setPower(leftBeltPower);
        rightBeltMotor.setPower(rightBeltPower);
    }

    private void handleIntakes() {
        double leftIntakePower = gamepad2.left_stick_y;
        leftIntakeLeftServo.setPower(leftIntakePower);
        leftIntakeRightServo.setPower(leftIntakePower);

        double rightIntakePower = gamepad2.right_stick_y;
        rightIntakeLeftServo.setPower(-rightIntakePower);
        rightIntakeRightServo.setPower(rightIntakePower);
    }

    // ============================================================
    // COLOR SENSORS + LIGHTS UPDATE (added)
    // ============================================================
    private void updateColorSensorsAndLights() {

        // Read color results
        String leftResult = classifyColor(leftColor, leftDistance, leftHsv);
        String rightResult = classifyColor(rightColor, rightDistance, rightHsv);

        // Decide per-side mode based on YOUR existing controls:
        // - INTAKE: stick up (negative) beyond deadband, AND not currently launching with bumper
        // - LAUNCH: bumper pressed
        // - IDLE otherwise
        SideMode leftMode = sideMode(gamepad2.left_stick_y, gamepad2.left_bumper);
        SideMode rightMode = sideMode(gamepad2.right_stick_y, gamepad2.right_bumper);

        // Intake: add events
        if (leftMode == SideMode.INTAKE) updateEventLoggerSide(leftResult, true);
        else if (leftMode == SideMode.LAUNCH) updateLaunchRemovalSide(leftResult, leftLog, true);
        else resetEventTrackingSide(true);

        if (rightMode == SideMode.INTAKE) updateEventLoggerSide(rightResult, false);
        else if (rightMode == SideMode.LAUNCH) updateLaunchRemovalSide(rightResult, rightLog, false);
        else resetEventTrackingSide(false);

        // Flash trigger (both sides) on gamepad2.x
        boolean x = gamepad2.x;
        if (x && !lastX) {
            if (!leftLog.isEmpty()) { leftFlashing = true; leftFlashStartSec = getRuntime(); }
            if (!rightLog.isEmpty()) { rightFlashing = true; rightFlashStartSec = getRuntime(); }
        }
        lastX = x;

        // Default = show NEXT launch
        double leftDefaultPos = posForColor(peekLaunchNext(leftLog));
        double rightDefaultPos = posForColor(peekLaunchNext(rightLog));

        double leftPos = leftDefaultPos;
        double rightPos = rightDefaultPos;

        // Flash overrides
        if (leftFlashing) {
            double p = computeFlashPos(leftLog, leftFlashStartSec);
            if (p < -0.5) { leftFlashing = false; leftPos = leftDefaultPos; }
            else leftPos = p;
        }
        if (rightFlashing) {
            double p = computeFlashPos(rightLog, rightFlashStartSec);
            if (p < -0.5) { rightFlashing = false; rightPos = rightDefaultPos; }
            else rightPos = p;
        }

        LeftLight.setPosition(leftPos);
        RightLight.setPosition(rightPos);

        // Optional debug (remove later)
        telemetry.addData("L Color", leftResult);
        telemetry.addData("R Color", rightResult);
        telemetry.addData("L Next", peekLaunchNext(leftLog));
        telemetry.addData("R Next", peekLaunchNext(rightLog));
    }

    private enum SideMode { IDLE, INTAKE, LAUNCH }

    private SideMode sideMode(double stickY, boolean launchBumper) {
        if (launchBumper) return SideMode.LAUNCH;
        // stick up = negative
        if (stickY < -0.6) return SideMode.INTAKE;
        return SideMode.IDLE;
    }

    private String classifyColor(NormalizedColorSensor color, DistanceSensor distance, float[] hsvOut) {
        double d = distance.getDistance(DistanceUnit.CM);
        if (Double.isNaN(d) || d > COLOR_CAP_CM) return "TOO FAR";

        NormalizedRGBA c = color.getNormalizedColors();
        Color.RGBToHSV(
                (int)(c.red * 255),
                (int)(c.green * 255),
                (int)(c.blue * 255),
                hsvOut
        );
        float h = hsvOut[0];

        if (h >= GREEN_MIN && h <= GREEN_MAX) return "GREEN";
        if (h >= PURPLE_MIN && h <= PURPLE_MAX) return "PURPLE";
        return "UNKNOWN";
    }

    // ---- Intake logging (per-side), allows same color again after a break ----
    private void updateEventLoggerSide(String result, boolean isLeft) {

        boolean isValid = result.equals("GREEN") || result.equals("PURPLE");

        if (isLeft) {
            if (!isValid) {
                leftStableCount = 0;
                leftLastSeen = "NONE";
                leftBreakCount++;
                if (leftBreakCount >= BREAK_LOOPS_REQUIRED) leftLoggedThisEvent = false;
                return;
            }

            leftBreakCount = 0;

            if (result.equals(leftLastSeen)) leftStableCount++;
            else { leftLastSeen = result; leftStableCount = 1; }

            if (!leftLoggedThisEvent && leftStableCount >= STABLE_LOOPS_REQUIRED) {
                pushLog(leftLog, result);
                leftLoggedThisEvent = true;
            }

        } else {
            if (!isValid) {
                rightStableCount = 0;
                rightLastSeen = "NONE";
                rightBreakCount++;
                if (rightBreakCount >= BREAK_LOOPS_REQUIRED) rightLoggedThisEvent = false;
                return;
            }

            rightBreakCount = 0;

            if (result.equals(rightLastSeen)) rightStableCount++;
            else { rightLastSeen = result; rightStableCount = 1; }

            if (!rightLoggedThisEvent && rightStableCount >= STABLE_LOOPS_REQUIRED) {
                pushLog(rightLog, result);
                rightLoggedThisEvent = true;
            }
        }
    }

    // ---- Launch removal (per-side): when next color stops being seen, removeLast() ----
    private void updateLaunchRemovalSide(String sensorResult, ArrayDeque<String> log, boolean isLeft) {
        String next = peekLaunchNext(log);
        if (next.equals("NONE")) {
            if (isLeft) { leftWasSeeingLaunchColor = false; leftLaunchGapCount = 0; }
            else { rightWasSeeingLaunchColor = false; rightLaunchGapCount = 0; }
            return;
        }

        boolean seeingNext = sensorResult.equals(next);

        if (isLeft) {
            if (seeingNext) {
                leftWasSeeingLaunchColor = true;
                leftLaunchGapCount = 0;
            } else if (leftWasSeeingLaunchColor) {
                leftLaunchGapCount++;
                if (leftLaunchGapCount >= LAUNCH_GAP_LOOPS_REQUIRED) {
                    log.removeLast();
                    leftWasSeeingLaunchColor = false;
                    leftLaunchGapCount = 0;
                }
            } else {
                leftLaunchGapCount = 0;
            }
        } else {
            if (seeingNext) {
                rightWasSeeingLaunchColor = true;
                rightLaunchGapCount = 0;
            } else if (rightWasSeeingLaunchColor) {
                rightLaunchGapCount++;
                if (rightLaunchGapCount >= LAUNCH_GAP_LOOPS_REQUIRED) {
                    log.removeLast();
                    rightWasSeeingLaunchColor = false;
                    rightLaunchGapCount = 0;
                }
            } else {
                rightLaunchGapCount = 0;
            }
        }
    }

    private void resetEventTrackingSide(boolean isLeft) {
        if (isLeft) {
            leftLastSeen = "NONE";
            leftStableCount = 0;
            leftBreakCount = BREAK_LOOPS_REQUIRED;
            leftLoggedThisEvent = false;
        } else {
            rightLastSeen = "NONE";
            rightStableCount = 0;
            rightBreakCount = BREAK_LOOPS_REQUIRED;
            rightLoggedThisEvent = false;
        }
    }

    private void pushLog(ArrayDeque<String> log, String value) {
        while (log.size() >= MAX_LOG) log.removeFirst();
        log.addLast(value);
    }

    private String peekLaunchNext(ArrayDeque<String> log) {
        if (log.isEmpty()) return "NONE";
        return log.peekLast(); // newest = next to launch
    }

    private double posForColor(String colorName) {
        if (colorName.equals("GREEN")) return GREEN_POS;
        if (colorName.equals("PURPLE")) return PURPLE_POS;
        if (colorName.equals("NONE")) return UNKNOWN_POS;
        return UNKNOWN_POS;
    }

    /**
     * Flash sequence (variable durations):
     *  - Yellow (pre) 0.25s
     *  - Color (0.50s), Yellow (0.25s), Color (0.50s), Yellow (0.25s), ...
     *  - Yellow (post) 0.25s
     *
     * Launch order is newest->oldest = deque last->first.
     * Returns -1 when done.
     */
    private double computeFlashPos(ArrayDeque<String> log, double startSec) {
        int n = log.size();
        if (n <= 0) return -1;

        double t = getRuntime() - startSec;

        // Pre-yellow
        if (t < YELLOW_SEC) return YELLOW_POS;
        t -= YELLOW_SEC;

        // For each color: color (0.50s) then yellow (0.25s)
        for (int i = 0; i < n; i++) {

            if (t < COLOR_SEC) {
                String colorName = getLaunchOrderItem(log, i); // i from newest
                return posForColor(colorName);
            }
            t -= COLOR_SEC;

            if (t < YELLOW_SEC) return YELLOW_POS;
            t -= YELLOW_SEC;
        }

        // Post-yellow
        if (t < YELLOW_SEC) return YELLOW_POS;

        return -1;
    }

    private String getLaunchOrderItem(ArrayDeque<String> log, int indexFromNewest) {
        int target = log.size() - 1 - indexFromNewest;
        int i = 0;
        for (String s : log) {
            if (i == target) return s;
            i++;
        }
        return "NONE";
    }
    // ============================================================

    private double zoneVelOut = 0.0;      // zone target velocity
    private double finalVelOut = 0.0;     // scaled target velocity
    private double leftVelOut = 0.0;      // measured
    private double rightVelOut = 0.0;     // measured

    private void handleLauncherPower(double tagHeight) {

        boolean upPressed = gamepad2.dpad_up && !lastDpadUp;
        boolean downPressed = gamepad2.dpad_down && !lastDpadDown;

        if (upPressed) noTagVelocity += NO_TAG_STEP;
        if (downPressed) noTagVelocity -= NO_TAG_STEP;

        lastDpadUp = gamepad2.dpad_up;
        lastDpadDown = gamepad2.dpad_down;

        if (gamepad2.dpad_left) launcherDirection = -1;
        else if (gamepad2.dpad_right) launcherDirection = 1;

        zoneVelOut = pickVelocityFromTagHeight(tagHeight);
        finalVelOut = zoneVelOut;

        boolean fire = (gamepad2.left_trigger > 0.05) || (gamepad2.right_trigger > 0.05);

        if (fire) {
            leftLauncher.setVelocity((finalVelOut + 150) * launcherDirection);
            rightLauncher.setVelocity((-finalVelOut - 150) * launcherDirection);
        } else {
            leftLauncher.setVelocity(0);
            rightLauncher.setVelocity(0);
        }

        leftVelOut = leftLauncher.getVelocity();
        rightVelOut = rightLauncher.getVelocity();
    }

    private void updateTelemetry(double tagHeight) {
        telemetry.addData("Drive Power", getDrivePowerModifier());
        telemetry.addData("Tag Height", tagHeight);
        telemetry.addData("LeftTop",  gamepad2.a ? "DOWN" :
                (Math.abs(gamepad2.left_stick_y) > INTAKE_DEADBAND ? "UP" : "MID"));
        telemetry.addData("RightTop", gamepad2.b ? "DOWN" :
                (Math.abs(gamepad2.right_stick_y) > INTAKE_DEADBAND ? "UP" : "MID"));
        boolean tagVisible = tagHeight >= 0;
        telemetry.addData("Tag", tagVisible ? "FOUND" : "NOT FOUND");
        telemetry.addData("ZoneVel(tps)", "%.0f", zoneVelOut);
        telemetry.addData("TargetVel(tps)", "%.0f", finalVelOut);
        telemetry.addData("LeftVel(tps)", "%.0f", leftVelOut);
        telemetry.addData("RightVel(tps)", "%.0f", rightVelOut);
        telemetry.update();
    }
}