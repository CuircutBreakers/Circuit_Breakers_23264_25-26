package org.firstinspires.ftc.teamcode.Teleop;

import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.PoseStorage;

@TeleOp
public class FullBlueTest extends LinearOpMode {

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
    private double launchPowerScale = 1.0;
    private int launcherDirection = 1; // 1 = normal, -1 = reversed

    // Right top intake positions
    private static final double RIGHT_UP_POS     = 0.51;
    private static final double RIGHT_DOWN_POS   = 0.472;
    private static final double RIGHT_MIDDLE_POS = 0.485;

    // Left top intake positions
    private static final double LEFT_UP_POS     = 0.475;
    private static final double LEFT_DOWN_POS   = 0.51;
    private static final double LEFT_MIDDLE_POS = 0.5;

    // Delay before settling to middle after release (ms)

    // State tracking

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



    // pixels of hysteresis (tune)

    private static final double INTAKE_DEADBAND = 0.05;

    final int TargetTagID = 4;



    @Override
    public void runOpMode() throws InterruptedException {
        initHardware();
        initDriveDirectionsAndBrakes();

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

// Flywheels usually feel better with FLOAT
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
                gamepad2.right_stick_y,   // right intake joystick
                gamepad2.b,               // button forces DOWN
                RIGHT_UP_POS,
                RIGHT_DOWN_POS,
                RIGHT_MIDDLE_POS
        );
    }

    private void TopLeftIntake() {
        updateTopIntakeServo(
                leftIntakeTopServo,
                gamepad2.left_stick_y,    // left intake joystick
                gamepad2.a,               // button forces DOWN
                LEFT_UP_POS,
                LEFT_DOWN_POS,
                LEFT_MIDDLE_POS
        );
    }


    private double pickVelocityFromTagHeight(double tagHeight) {
        if (tagHeight < 0) return 1650; // no tag

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
        // Direction config (adjust if needed for your wiring)
        frontLeft.setDirection(DcMotor.Direction.FORWARD);
        backLeft.setDirection(DcMotor.Direction.FORWARD);
        frontRight.setDirection(DcMotor.Direction.REVERSE);
        backRight.setDirection(DcMotor.Direction.REVERSE);

        // Brake mode for better control when sticks are released
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
        // GAMEPAD 1: MECANUM DRIVE
        double y  = -gamepad1.left_stick_y;   // forward is negative on stick
        double x  =  gamepad1.left_stick_x;   // strafe
        double rx =  gamepad1.right_stick_x;  // rotation

        // Normalize
        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1.0);
        double fl = (y + x + rx) / denominator;
        double bl = (y - x + rx) / denominator;
        double fr = (y - x - rx) / denominator;
        double br = (y + x - rx) / denominator;

        double drivePower = getDrivePowerModifier(); // from gamepad2 dpad up/down (your current behavior)

        frontLeft.setPower(fl * drivePower);
        backLeft.setPower(bl * drivePower);
        frontRight.setPower(fr * drivePower);
        backRight.setPower(br * drivePower);
    }

    private double getDrivePowerModifier() {
        // Your current logic: gamepad2 dpad_down = 0.5, dpad_up = 1.0, otherwise default 1.0
        if (gamepad1.left_bumper) return 0.5;
        return 1.0;

    }

    private void handleBelts() {
        // Left belt
        double leftBeltPower;
        if (gamepad2.left_bumper) {
            leftBeltPower = 1.0;
        } else if (gamepad2.left_stick_y < 0) {
            leftBeltPower = -1.0;
        } else {
            leftBeltPower = 0.0;
        }

        // Right belt
        double rightBeltPower;
        if (gamepad2.right_bumper) {
            rightBeltPower = 1.0;
        } else if (gamepad2.right_stick_y < 0) {
            rightBeltPower = -1.0;
        } else {
            rightBeltPower = 0.0;
        }

        leftBeltMotor.setPower(leftBeltPower);
        rightBeltMotor.setPower(rightBeltPower);
    }

    private void handleIntakes() {

        // Left intake (all three CRServos same power)
        double leftIntakePower = gamepad2.left_stick_y;
        leftIntakeLeftServo.setPower(leftIntakePower);
        leftIntakeRightServo.setPower(leftIntakePower);

        // Right intake (note your original sign convention)
        double rightIntakePower = gamepad2.right_stick_y;
        rightIntakeLeftServo.setPower(-rightIntakePower);
        rightIntakeRightServo.setPower(rightIntakePower);
    }





    private double zoneVelOut = 0.0;      // zone target velocity
    private double finalVelOut = 0.0;     // scaled target velocity
    private double leftVelOut = 0.0;      // measured
    private double rightVelOut = 0.0;     // measured

    private void handleLauncherPower(double tagHeight) {

        // Direction toggle
        if (gamepad2.dpad_left) {
            launcherDirection = -1;
        } else if (gamepad2.dpad_right) {
            launcherDirection = 1;
        }

        // Manual scale knob (same behavior, but scaling velocity now)
        if (gamepad2.dpad_down) {
            launchPowerScale = 0.95;
        } else if (gamepad2.dpad_up) {
            launchPowerScale = 1.00;
        }

        // 1) Zone velocity from tag height
        zoneVelOut = pickVelocityFromTagHeight(tagHeight);

        // 2) Apply manual scaling
        finalVelOut = zoneVelOut * launchPowerScale;

        boolean fire = (gamepad2.left_trigger > 0.05) || (gamepad2.right_trigger > 0.05);

        if (fire) {
            // Keep your original sign convention: right is negative relative to left
            leftLauncher.setVelocity( (finalVelOut +150) * launcherDirection);
            rightLauncher.setVelocity((-finalVelOut - 150) * launcherDirection);
        } else {
            leftLauncher.setVelocity(0);
            rightLauncher.setVelocity(0);
        }

        // Read actual velocity for telemetry
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
        telemetry.addData("Launch Scale", launchPowerScale);
        telemetry.addData("ZoneVel(tps)", "%.0f", zoneVelOut);
        telemetry.addData("TargetVel(tps)", "%.0f", finalVelOut);
        telemetry.addData("LeftVel(tps)", "%.0f", leftVelOut);
        telemetry.addData("RightVel(tps)", "%.0f", rightVelOut);
        telemetry.update();
    }



}
