package org.firstinspires.ftc.teamcode.Teleop;

import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.PoseStorage;

@TeleOp
public class FullRed extends LinearOpMode {

    // =========================
    // Hardware (class fields)
    // =========================
    private DcMotor frontLeft, frontRight, backLeft, backRight;
    private DcMotor leftLauncher, rightLauncher;

    private CRServo leftIntakeLeftServo, leftIntakeRightServo, leftBeltServo;
    private CRServo rightIntakeLeftServo, rightIntakeRightServo, rightBeltServo;
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
    private static final double P_FAR    = .79; // height < H1
    private static final double P_MID1   = 0.78; // H1..H2
    private static final double P_MID2   = 0.76; // H2..H3
    private static final double P_MID3   = 0.75; // H3..H4
    private static final double P_NEAR   = 0.725; // height >= H4


    // Voltage compensation steps (tune)
    private static final double V1 = 13.8;  // fresh
    private static final double V2 = 13.6;
    private static final double V3 = 13.4;
    private static final double V4 = 13.2;  // getting low

    // Multipliers (higher when voltage is lower)
    private static final double VC_FRESH = 1.00;  // >= V1
    private static final double VC_MID1  = 1.03;  // V2..V1
    private static final double VC_MID2  = 1.06;  // V3..V2
    private static final double VC_MID3  = 1.09;  // V4..V3
    private static final double VC_LOW   = 1.12;  // < V4

    // Over-voltage reduction (tune)
    private static final double OV1 = 14.0;  // start reducing above this
    private static final double OV2 = 14.2;
    private static final double OV3 = 14.4;

    // Multipliers (lower when voltage is too high)
    private static final double OV_OK     = 1.00; // <= OV1
    private static final double OV_HIGH1  = 0.97; // OV1..OV2
    private static final double OV_HIGH2  = 0.94; // OV2..OV3
    private static final double OV_HIGH3  = 0.91; // > OV3

    private double overVoltOut = 1.0;


    private double zonePowerOut = 0.0;       // distance step power
    private double voltCompOut = 1.0;        // voltage step multiplier
    private double finalPowerOut = 0.0;      // final applied power (after scale + volt comp)
    private double batteryVOut = 0.0;        // optional, nice to display

    private double motorPowerOut = 0.0;

    // pixels of hysteresis (tune)

    private static final double INTAKE_DEADBAND = 0.05;

    final int TargetTagID = 5;



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
        leftLauncher  = hardwareMap.get(DcMotor.class, "LeftLauncher");
        rightLauncher = hardwareMap.get(DcMotor.class, "RightLauncher");

        // Left intake + belt
        leftIntakeLeftServo  = hardwareMap.get(CRServo.class, "LeftIntakeLeftServo");
        leftIntakeRightServo = hardwareMap.get(CRServo.class, "LeftIntakeRightServo");
        leftIntakeTopServo   = hardwareMap.get(Servo.class, "LeftIntakeTopServo");
        leftBeltServo        = hardwareMap.get(CRServo.class, "LeftBeltServo");

        // Right intake + belt
        rightIntakeLeftServo  = hardwareMap.get(CRServo.class, "RightIntakeLeftServo");
        rightIntakeRightServo = hardwareMap.get(CRServo.class, "RightIntakeRightServo");
        rightIntakeTopServo   = hardwareMap.get(Servo.class, "RightIntakeTopServo");
        rightBeltServo        = hardwareMap.get(CRServo.class, "RightBeltServo");

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


    private double pickPowerFromTagHeight(double tagHeight) {
        if (tagHeight < 0) return .70; // no tag

        if (tagHeight < H1) return P_FAR;
        if (tagHeight < H2) return P_MID1;
        if (tagHeight < H3) return P_MID2;
        if (tagHeight < H4) return P_MID3;
        return P_NEAR;
    }
    private double pickVoltageCompStep(double batteryV) {
        if (batteryV >= V1) return VC_FRESH;
        if (batteryV >= V2) return VC_MID1;
        if (batteryV >= V3) return VC_MID2;
        if (batteryV >= V4) return VC_MID3;
        return VC_LOW;
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
            leftBeltPower = -1.0;
        } else if (gamepad2.left_stick_y < 0) {
            leftBeltPower = 1.0;
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

        leftBeltServo.setPower(leftBeltPower);
        rightBeltServo.setPower(rightBeltPower);
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

    private double getBatteryVoltage() {
        double minV = Double.POSITIVE_INFINITY;
        for (VoltageSensor sensor : hardwareMap.voltageSensor) {
            double v = sensor.getVoltage();
            if (v > 0) minV = Math.min(minV, v);
        }
        return (minV == Double.POSITIVE_INFINITY) ? 12.0 : minV;
    }



    private double pickOverVoltageReduction(double batteryV) {
        if (batteryV <= OV1) return OV_OK;
        if (batteryV <= OV2) return OV_HIGH1;
        if (batteryV <= OV3) return OV_HIGH2;
        return OV_HIGH3;
    }




    private void handleLauncherPower(double tagHeight) {

        // Direction toggle
        if (gamepad2.dpad_left) {
            launcherDirection = -1;
        } else if (gamepad2.dpad_right) {
            launcherDirection = 1;
        }

        // Manual scale knob
        if (gamepad2.dpad_down) {
            launchPowerScale = 0.95;
        } else if (gamepad2.dpad_up) {
            launchPowerScale = 1.00;
        }

        // 1) Zone power (distance steps)
        zonePowerOut = pickPowerFromTagHeight(tagHeight);

        // 2) Voltage step multiplier
        batteryVOut = getBatteryVoltage();
        voltCompOut = pickVoltageCompStep(batteryVOut);
        overVoltOut = pickOverVoltageReduction(batteryVOut);

// Final power (zone * scale * voltage comp * over-voltage reduction)
        finalPowerOut = zonePowerOut * launchPowerScale * voltCompOut * overVoltOut;

        // Clamp to legal motor power range
        finalPowerOut = Math.max(0.0, Math.min(1.0, finalPowerOut));

        boolean fire = (gamepad2.left_trigger > 0.05) || (gamepad2.right_trigger > 0.05);

        if (fire) {
            motorPowerOut = finalPowerOut;
            leftLauncher.setPower(finalPowerOut * launcherDirection);
            rightLauncher.setPower(-finalPowerOut * launcherDirection);
        } else {
            motorPowerOut = 0.0;
            leftLauncher.setPower(0);
            rightLauncher.setPower(0);
        }
    }


    private void updateTelemetry(double tagHeight) {
        telemetry.addData("position", drive.localizer.getPose());
        telemetry.addData("Drive Power", getDrivePowerModifier());
        telemetry.addData("Launch Power", launchPowerScale);
        telemetry.addData("BatteryV", "%.2f", batteryVOut);
        telemetry.addData("ZonePower", "%.3f", zonePowerOut);
        telemetry.addData("VoltAdj", "%.3f", voltCompOut);
        telemetry.addData("FinalPower", "%.3f", finalPowerOut);
        telemetry.addData("MotorPower", "%.3f", motorPowerOut);
        telemetry.addData("Tag Height", tagHeight);
        telemetry.addData("OverVoltAdj", "%.3f", overVoltOut);
        telemetry.addData("LeftTop",  gamepad2.a ? "DOWN" :
                (Math.abs(gamepad2.left_stick_y) > INTAKE_DEADBAND ? "UP" : "MID"));
        telemetry.addData("RightTop", gamepad2.b ? "DOWN" :
                (Math.abs(gamepad2.right_stick_y) > INTAKE_DEADBAND ? "UP" : "MID"));
        boolean tagVisible = tagHeight >= 0;
        telemetry.addData("Tag", tagVisible ? "FOUND" : "NOT FOUND");
        telemetry.update();
    }



}
