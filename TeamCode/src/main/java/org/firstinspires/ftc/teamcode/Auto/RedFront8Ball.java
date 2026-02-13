package org.firstinspires.ftc.teamcode.Auto;


import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.MecanumDrive;

@Autonomous
public class RedFront8Ball extends LinearOpMode {
    double AprilTagDetected = 0;
    double DetectedTagID = 0;

    HuskyLens Camera;
    // --- Voltage compensation (simple, read once) ---
    private static final double V_TARGET = 13.7;   // tune: voltage you expect during good shooting
    private static final double MULT_MIN = 0.95;   // don't reduce more than this
    private static final double MULT_MAX = 1.10;   // don't boost more than this

    private double lastLeftLauncherPower = 0.0;
    private double lastRightLauncherPower = 0.0;


    private double getBatteryVoltageOnce() {
        double minV = Double.POSITIVE_INFINITY;
        for (VoltageSensor sensor : hardwareMap.voltageSensor) {
            double v = sensor.getVoltage();
            if (v > 0) minV = Math.min(minV, v);
        }
        return (minV == Double.POSITIVE_INFINITY) ? 12.0 : minV;
    }

    private static double clamp(double x, double lo, double hi) {
        return Math.max(lo, Math.min(hi, x));
    }

    private static double computeVoltMult(double v) {
        // Simple proportional compensation:
        // If v is low -> multiplier >1, if v is high -> multiplier <1
        // Example: v=13.0 => 13.8/13.0 = 1.0615
        double mult = V_TARGET / v;
        return clamp(mult, MULT_MIN, MULT_MAX);
    }


    // Over-voltage reduction (tune)
    // Voltage compensation (tune)
// Target band ~13.8V (typical "good" under load)

    @Override
    public void runOpMode() throws InterruptedException {

        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(52,-53,Math.toRadians(-51)));
        DcMotor LeftLauncher = hardwareMap.get(DcMotor.class, "LeftLauncher");
        DcMotor RightLauncher = hardwareMap.get(DcMotor.class, "RightLauncher");
        DcMotor frontLeft = hardwareMap.get(DcMotor.class, "leftFront");
        DcMotor frontRight = hardwareMap.get(DcMotor.class, "rightFront");
        DcMotor backLeft = hardwareMap.get(DcMotor.class, "leftBack");
        DcMotor backRight = hardwareMap.get(DcMotor.class, "rightBack");

        CRServo RightIntakeLeftServo = hardwareMap.get(CRServo.class, "RightIntakeLeftServo");
        CRServo RightIntakeRightServo = hardwareMap.get(CRServo.class, "RightIntakeRightServo");
        Servo LeftTopServo = hardwareMap.get(Servo.class, "LeftIntakeTopServo");
        CRServo LeftIntakeLeftServo = hardwareMap.get(CRServo.class, "LeftIntakeLeftServo");
        CRServo LeftIntakeRightServo = hardwareMap.get(CRServo.class, "LeftIntakeRightServo");
        Servo RightTopServo = hardwareMap.get(Servo.class, "RightIntakeTopServo");
        CRServo LeftBeltServo = hardwareMap.get(CRServo.class, "LeftBeltServo");
        CRServo RightBeltServo = hardwareMap.get(CRServo.class, "RightBeltServo");

        Camera = hardwareMap.get(HuskyLens.class, "huskylens");
        Camera.selectAlgorithm(HuskyLens.Algorithm.TAG_RECOGNITION);
        VoltageSensor battSensor = hardwareMap.voltageSensor.iterator().next();

        //Intake pushers Init
        RightTopServo.setPosition(.485);
        LeftTopServo.setPosition(.5);
        //Ball position Init
        LeftBeltServo.setPower(1);
        RightBeltServo.setPower(-1);
        sleep(500);
        LeftBeltServo.setPower(0);
        RightBeltServo.setPower(0);

        waitForStart();

        double battV = getBatteryVoltageOnce();
        double voltMult = computeVoltMult(battV);

        telemetry.addData("battV", battV);
        telemetry.addData("voltMult", voltMult);
        telemetry.update();

        Actions.runBlocking(
                drive.actionBuilder(new Pose2d(52,-53,Math.toRadians(-51)))
                        .stopAndAdd(new PrimeLaunch(LeftLauncher,RightLauncher, .75,.775,0,voltMult))
                        .strafeToLinearHeading(new Vector2d(25,-25), Math.toRadians(15))
                        .build()
        );
        ElapsedTime TagTime;
        TagTime = new ElapsedTime();
        while (AprilTagDetected == 0) {
            HuskyLens.Block[] blocks = Camera.blocks();

            if (blocks.length ==1) {
                DetectedTagID = blocks[0].id;
            } else if (TagTime.seconds() > .5){
                DetectedTagID = 1;
            }
            AprilTagDetected = DetectedTagID;
            telemetry.addData("LeftLauncherPower", lastLeftLauncherPower);
            telemetry.addData("RightLauncherPower", lastRightLauncherPower);
            telemetry.addData("April Tag Detected", AprilTagDetected);
            telemetry.update();
        }
        if (AprilTagDetected == 1){
            Actions.runBlocking(
                    drive.actionBuilder(new Pose2d(25,-25,Math.toRadians(15)))
                            .strafeToLinearHeading(new Vector2d(28,-25), Math.toRadians(-43))
                            .stopAndAdd(new StartBelt(RightBeltServo,-1))
                            .waitSeconds(.4)
                            .turnTo(Math.toRadians(-53))
                            .stopAndAdd(new LaunchLeft(LeftBeltServo, .1,-1))
                            .stopAndAdd(new StartBelt(LeftBeltServo,1))
                            .waitSeconds(2)
                            .stopAndAdd(new RightIntakeStart(RightIntakeLeftServo,RightIntakeRightServo,RightBeltServo,1))
                            .stopAndAdd(new TopRightIntake(RightTopServo,3))
                            //line up for ball 1
                            .strafeToLinearHeading(new Vector2d(15,-30), Math.toRadians(0))
                            .stopAndAdd(new StopLaunch(LeftLauncher, RightLauncher))
                            .stopAndAdd(new StrafeLeftSlow(frontLeft,frontRight,backLeft,backRight,-.3,.5))
                            .stopAndAdd(new StrafeLeftSlow(frontLeft,frontRight,backLeft,backRight,0,.15))
                            .stopAndAdd(new TopRightIntake(RightTopServo, 1))
                            .waitSeconds(.75)
                            .stopAndAdd(new TopRightIntake(RightTopServo,3))
                            .stopAndAdd(new StrafeLeftSlow(frontLeft,frontRight,backLeft,backRight,-.25,1))
                            .stopAndAdd(new StrafeLeftSlow(frontLeft,frontRight,backLeft,backRight,1,.4))
                            .stopAndAdd(new TopRightIntake(RightTopServo, 1))
                            .stopAndAdd(new TopLeftIntake(LeftTopServo,3))
                            .stopAndAdd(new TopRightIntake(RightTopServo,2))
                            .stopAndAdd(new IntakeStop(RightIntakeLeftServo,RightIntakeRightServo,RightBeltServo))
                            .stopAndAdd(new StartBelt(RightBeltServo,.125))
                            .stopAndAdd(new LeftIntakeStart(LeftIntakeLeftServo,LeftIntakeRightServo,LeftBeltServo,1))
                            .build()
            );
            drive.updatePoseEstimate();

            Actions.runBlocking(
                    drive.actionBuilder(drive.localizer.getPose())
                            .strafeToLinearHeading(new Vector2d(19,-38),Math.toRadians(180))
                            .stopAndAdd(new StartBelt(RightBeltServo,0))
                            .stopAndAdd(new StrafeLeftSlow(frontLeft,frontRight,backLeft,backRight,1,.6))
                            .stopAndAdd(new StrafeLeftSlow(frontLeft,frontRight,backLeft,backRight,-1,.2))
                            .stopAndAdd(new TopLeftIntake(LeftTopServo,1))
                            .build()
            );
            drive.updatePoseEstimate();
            Actions.runBlocking(
                    drive.actionBuilder(drive.localizer.getPose())
                            .strafeToLinearHeading(new Vector2d(20, -32), Math.toRadians(-23))
                            .stopAndAdd(new TopLeftIntake(LeftTopServo,2))
                            .stopAndAdd(new StartBelt(LeftBeltServo,.25))
                            .stopAndAdd(new PrimeLaunch(LeftLauncher,RightLauncher,.775,.775,0,voltMult))
                            .strafeToLinearHeading(new Vector2d(33,-22), Math.toRadians(-52))
                            .stopAndAdd(new IntakeStop(LeftIntakeLeftServo,LeftIntakeRightServo,LeftBeltServo))
                            .stopAndAdd(new StartBelt(LeftBeltServo,1))
                            .waitSeconds(.5)
                            .turnTo(Math.toRadians(-43))
                            .stopAndAdd(new StartBelt(RightBeltServo,-1))
                            .waitSeconds(2.75)
                            .stopAndAdd(new StopLaunch(LeftLauncher,RightLauncher))
                            .stopAndAdd(new StartBelt(LeftBeltServo,0))
                            .stopAndAdd(new StartBelt(RightBeltServo,0))
                            .stopAndAdd(new RightIntakeStart(RightIntakeLeftServo,RightIntakeRightServo,RightBeltServo,-1))
                            .stopAndAdd(new TopRightIntake(RightTopServo, 3))
                            .strafeToLinearHeading(new Vector2d(-6,-30),Math.toRadians(0))
                            .build()
            );
            drive.updatePoseEstimate();
            Actions.runBlocking(
                    drive.actionBuilder(drive.localizer.getPose())
                            .stopAndAdd(new RightIntakeStart(RightIntakeLeftServo,RightIntakeRightServo,RightBeltServo,1))
                            .strafeToLinearHeading(new Vector2d(-6,-30), Math.toRadians(0))
                            .stopAndAdd(new StrafeLeftSlow(frontLeft,frontRight,backLeft,backRight,-1,.1))
                            .stopAndAdd(new StrafeLeftSlow(frontLeft,frontRight,backLeft,backRight,-.3,.5))
                            .stopAndAdd(new StrafeLeftSlow(frontLeft,frontRight,backLeft,backRight,0,.15))
                            .stopAndAdd(new TopRightIntake(RightTopServo, 1))
                            .waitSeconds(.3)
                            .stopAndAdd(new TopRightIntake(RightTopServo, 3))
                            .stopAndAdd(new StrafeLeftSlow(frontLeft,frontRight,backLeft,backRight,-.3,1))
                            .stopAndAdd(new StrafeLeftSlow(frontLeft,frontRight,backLeft,backRight,0,.25))
                            .stopAndAdd(new TopRightIntake(RightTopServo, 1))
                            .strafeToLinearHeading(new Vector2d(12,-18), Math.toRadians(0))
                            .stopAndAdd(new TopRightIntake(RightTopServo, 3))
                            .stopAndAdd(new RightIntakeStart(RightIntakeLeftServo,RightIntakeRightServo,RightBeltServo,-1))
                            .stopAndAdd(new StartBelt(RightBeltServo,-.05))
                            .stopAndAdd(new PrimeLaunch(LeftLauncher,RightLauncher,0,.775,0,voltMult))
                            .strafeToLinearHeading(new Vector2d(47.5,-15), Math.toRadians(-58))
                            .stopAndAdd(new StartBelt(RightBeltServo,-1))
                            .waitSeconds(3.5)
                            .build()
            );
        }
        else if (AprilTagDetected == 2) {
            Actions.runBlocking(
                    drive.actionBuilder(new Pose2d(25,-25,Math.toRadians(15)))
                            .strafeToLinearHeading(new Vector2d(28,-25), Math.toRadians(-53))
                            .stopAndAdd(new LaunchLeft(LeftBeltServo, .1,-1))
                            .stopAndAdd(new StartBelt(LeftBeltServo,1))
                            .waitSeconds(.55)
                            .stopAndAdd(new StartBelt(RightBeltServo,-1))
                            .stopAndAdd(new StartBelt(LeftBeltServo,.75))
                            .turnTo(Math.toRadians(-43))
                            .waitSeconds(.2)
                            .stopAndAdd(new StartBelt(LeftBeltServo,1))
                            .turnTo(Math.toRadians(-53))
                            .stopAndAdd(new RightIntakeStart(RightIntakeLeftServo,RightIntakeRightServo,RightBeltServo,1))
                            .stopAndAdd(new TopRightIntake(RightTopServo,3))
                            //line up for ball 1
                            .strafeToLinearHeading(new Vector2d(15,-30), Math.toRadians(0))
                            .stopAndAdd(new StopLaunch(LeftLauncher, RightLauncher))
                            .stopAndAdd(new StrafeLeftSlow(frontLeft,frontRight,backLeft,backRight,-.3,.5))
                            .stopAndAdd(new StrafeLeftSlow(frontLeft,frontRight,backLeft,backRight,0,.15))
                            .stopAndAdd(new TopRightIntake(RightTopServo, 1))
                            .waitSeconds(.75)
                            .stopAndAdd(new TopRightIntake(RightTopServo,3))
                            .stopAndAdd(new StrafeLeftSlow(frontLeft,frontRight,backLeft,backRight,-.25,1))
                            .stopAndAdd(new StrafeLeftSlow(frontLeft,frontRight,backLeft,backRight,1,.4))
                            .stopAndAdd(new TopRightIntake(RightTopServo, 1))
                            .stopAndAdd(new TopLeftIntake(LeftTopServo,3))
                            .stopAndAdd(new TopRightIntake(RightTopServo,2))
                            .stopAndAdd(new IntakeStop(RightIntakeLeftServo,RightIntakeRightServo,RightBeltServo))
                            .stopAndAdd(new StartBelt(RightBeltServo,.125))
                            .stopAndAdd(new LeftIntakeStart(LeftIntakeLeftServo,LeftIntakeRightServo,LeftBeltServo,1))
                            .build()
            );
            drive.updatePoseEstimate();

            Actions.runBlocking(
                    drive.actionBuilder(drive.localizer.getPose())
                            .strafeToLinearHeading(new Vector2d(19,-38),Math.toRadians(180))
                            .stopAndAdd(new StartBelt(RightBeltServo,0))
                            .stopAndAdd(new StrafeLeftSlow(frontLeft,frontRight,backLeft,backRight,1,.6))
                            .stopAndAdd(new StrafeLeftSlow(frontLeft,frontRight,backLeft,backRight,-1,.2))
                            .stopAndAdd(new TopLeftIntake(LeftTopServo,1))
                            .build()
            );
            drive.updatePoseEstimate();
            Actions.runBlocking(
                    drive.actionBuilder(drive.localizer.getPose())
                            .strafeToLinearHeading(new Vector2d(20, -32), Math.toRadians(-23))
                            .stopAndAdd(new TopLeftIntake(LeftTopServo,2))
                            .stopAndAdd(new StartBelt(LeftBeltServo,.25))
                            .stopAndAdd(new PrimeLaunch(LeftLauncher,RightLauncher,.775,.775,0,voltMult))
                            .strafeToLinearHeading(new Vector2d(33,-22), Math.toRadians(-48))
                            .stopAndAdd(new IntakeStop(LeftIntakeLeftServo,LeftIntakeRightServo,LeftBeltServo))
                            .stopAndAdd(new StartBelt(RightBeltServo,-1))
                            .waitSeconds(.25)
                            .stopAndAdd(new StartBelt(LeftBeltServo,1))
                            .waitSeconds(2)
                            .stopAndAdd(new StopLaunch(LeftLauncher,RightLauncher))
                            .stopAndAdd(new StartBelt(LeftBeltServo,0))
                            .stopAndAdd(new StartBelt(RightBeltServo,0))
                            .stopAndAdd(new RightIntakeStart(RightIntakeLeftServo,RightIntakeRightServo,RightBeltServo,-1))
                            .stopAndAdd(new TopRightIntake(RightTopServo, 3))
                            .strafeToLinearHeading(new Vector2d(-6,-30),Math.toRadians(0))
                            .build()
            );
            drive.updatePoseEstimate();
            Actions.runBlocking(
                    drive.actionBuilder(drive.localizer.getPose())
                            .stopAndAdd(new RightIntakeStart(RightIntakeLeftServo,RightIntakeRightServo,RightBeltServo,1))
                            .strafeToLinearHeading(new Vector2d(-6,-30), Math.toRadians(0))
                            .stopAndAdd(new StrafeLeftSlow(frontLeft,frontRight,backLeft,backRight,-1,.1))
                            .stopAndAdd(new StrafeLeftSlow(frontLeft,frontRight,backLeft,backRight,-.3,.5))
                            .stopAndAdd(new StrafeLeftSlow(frontLeft,frontRight,backLeft,backRight,0,.15))
                            .stopAndAdd(new TopRightIntake(RightTopServo, 1))
                            .waitSeconds(.3)
                            .stopAndAdd(new TopRightIntake(RightTopServo, 3))
                            .stopAndAdd(new StrafeLeftSlow(frontLeft,frontRight,backLeft,backRight,-.5,1))
                            .stopAndAdd(new StrafeLeftSlow(frontLeft,frontRight,backLeft,backRight,0,.45))
                            .stopAndAdd(new TopRightIntake(RightTopServo, 1))
                            .strafeToLinearHeading(new Vector2d(12,-18), Math.toRadians(0))
                            .stopAndAdd(new TopRightIntake(RightTopServo, 3))
                            .stopAndAdd(new RightIntakeStart(RightIntakeLeftServo,RightIntakeRightServo,RightBeltServo,-1))
                            .stopAndAdd(new StartBelt(RightBeltServo,-.05))
                            .stopAndAdd(new PrimeLaunch(LeftLauncher,RightLauncher,0,.775,0,voltMult))
                            .strafeToLinearHeading(new Vector2d(47.5,-15), Math.toRadians(-58))
                            .stopAndAdd(new StartBelt(RightBeltServo,-1))
                            .waitSeconds(3.5)
                            .build()
            );

        }
        else if (AprilTagDetected == 3) {
            Actions.runBlocking(
                    drive.actionBuilder(new Pose2d(25,-25,Math.toRadians(15)))
                            .strafeToLinearHeading(new Vector2d(28,-25), Math.toRadians(-53))
                            .stopAndAdd(new LaunchLeft(LeftBeltServo, .1,-1))
                            .stopAndAdd(new StartBelt(LeftBeltServo,1))
                            .waitSeconds(1.75)
                            .turnTo(Math.toRadians(-40))
                            .stopAndAdd(new StartBelt(RightBeltServo,-1))
                            .waitSeconds(.75)
                            .stopAndAdd(new RightIntakeStart(RightIntakeLeftServo,RightIntakeRightServo,RightBeltServo,1))
                            .stopAndAdd(new TopRightIntake(RightTopServo,3))
                            //line up for ball 1
                            .strafeToLinearHeading(new Vector2d(15,-30), Math.toRadians(0))
                            .stopAndAdd(new StopLaunch(LeftLauncher, RightLauncher))
                            .stopAndAdd(new StrafeLeftSlow(frontLeft,frontRight,backLeft,backRight,-.3,.5))
                            .stopAndAdd(new StrafeLeftSlow(frontLeft,frontRight,backLeft,backRight,0,.15))
                            .stopAndAdd(new TopRightIntake(RightTopServo, 1))
                            .waitSeconds(.75)
                            .stopAndAdd(new TopRightIntake(RightTopServo,3))
                            .stopAndAdd(new StrafeLeftSlow(frontLeft,frontRight,backLeft,backRight,-.25,1))
                            .stopAndAdd(new StrafeLeftSlow(frontLeft,frontRight,backLeft,backRight,1,.4))
                            .stopAndAdd(new TopRightIntake(RightTopServo, 1))
                            .stopAndAdd(new TopLeftIntake(LeftTopServo,3))
                            .stopAndAdd(new TopRightIntake(RightTopServo,2))
                            .stopAndAdd(new IntakeStop(RightIntakeLeftServo,RightIntakeRightServo,RightBeltServo))
                            .stopAndAdd(new StartBelt(RightBeltServo,.125))
                            .stopAndAdd(new LeftIntakeStart(LeftIntakeLeftServo,LeftIntakeRightServo,LeftBeltServo,1))
                            .build()
            );
            drive.updatePoseEstimate();

            Actions.runBlocking(
                    drive.actionBuilder(drive.localizer.getPose())
                            .strafeToLinearHeading(new Vector2d(19,-38),Math.toRadians(180))
                            .stopAndAdd(new StartBelt(RightBeltServo,0))
                            .stopAndAdd(new StrafeLeftSlow(frontLeft,frontRight,backLeft,backRight,1,.6))
                            .stopAndAdd(new StrafeLeftSlow(frontLeft,frontRight,backLeft,backRight,-1,.2))
                            .stopAndAdd(new TopLeftIntake(LeftTopServo,1))
                            .build()
            );
            drive.updatePoseEstimate();
            Actions.runBlocking(
                    drive.actionBuilder(drive.localizer.getPose())
                            .strafeToLinearHeading(new Vector2d(20, -32), Math.toRadians(-23))
                            .stopAndAdd(new TopLeftIntake(LeftTopServo,2))
                            .stopAndAdd(new StartBelt(LeftBeltServo,.25))
                            .stopAndAdd(new PrimeLaunch(LeftLauncher,RightLauncher,.775,.775,0,voltMult))
                            .strafeToLinearHeading(new Vector2d(33,-22), Math.toRadians(-43))
                            .stopAndAdd(new IntakeStop(LeftIntakeLeftServo,LeftIntakeRightServo,LeftBeltServo))
                            .stopAndAdd(new StartBelt(RightBeltServo,1))
                            .waitSeconds(.15)
                            .stopAndAdd(new StartBelt(RightBeltServo,-1))
                            .stopAndAdd(new StartBelt(LeftBeltServo,.35))
                            .waitSeconds(1.75)
                            .turnTo(Math.toRadians(-53))
                            .stopAndAdd(new StartBelt(LeftBeltServo,1))
                            .waitSeconds(.25)
                            .stopAndAdd(new StopLaunch(LeftLauncher,RightLauncher))
                            .stopAndAdd(new StartBelt(LeftBeltServo,0))
                            .stopAndAdd(new StartBelt(RightBeltServo,0))
                            .stopAndAdd(new RightIntakeStart(RightIntakeLeftServo,RightIntakeRightServo,RightBeltServo,-1))
                            .stopAndAdd(new TopRightIntake(RightTopServo, 3))
                            .strafeToLinearHeading(new Vector2d(-6,-30),Math.toRadians(0))
                            .build()
            );
            drive.updatePoseEstimate();
            Actions.runBlocking(
                    drive.actionBuilder(drive.localizer.getPose())
                            .stopAndAdd(new RightIntakeStart(RightIntakeLeftServo,RightIntakeRightServo,RightBeltServo,1))
                            .strafeToLinearHeading(new Vector2d(-6,-30), Math.toRadians(0))
                            .stopAndAdd(new StrafeLeftSlow(frontLeft,frontRight,backLeft,backRight,-1,.1))
                            .stopAndAdd(new StrafeLeftSlow(frontLeft,frontRight,backLeft,backRight,-.3,.5))
                            .stopAndAdd(new StrafeLeftSlow(frontLeft,frontRight,backLeft,backRight,0,.15))
                            .stopAndAdd(new TopRightIntake(RightTopServo, 1))
                            .waitSeconds(.3)
                            .stopAndAdd(new TopRightIntake(RightTopServo, 3))
                            .stopAndAdd(new StrafeLeftSlow(frontLeft,frontRight,backLeft,backRight,-.4,.75))
                            .stopAndAdd(new StrafeLeftSlow(frontLeft,frontRight,backLeft,backRight,0,.25))
                            .stopAndAdd(new TopRightIntake(RightTopServo, 1))
                            .strafeToLinearHeading(new Vector2d(12,-18), Math.toRadians(0))
                            .stopAndAdd(new TopRightIntake(RightTopServo, 3))
                            .stopAndAdd(new RightIntakeStart(RightIntakeLeftServo,RightIntakeRightServo,RightBeltServo,-1))
                            .stopAndAdd(new StartBelt(RightBeltServo,.15))
                            .stopAndAdd(new PrimeLaunch(LeftLauncher,RightLauncher,0,.775,0,voltMult))
                            .stopAndAdd(new TopRightIntake(RightTopServo, 1))
                            .strafeToLinearHeading(new Vector2d(47.5,-15), Math.toRadians(-57))
                            .stopAndAdd(new TopRightIntake(RightTopServo, 3))
                            .stopAndAdd(new StartBelt(RightBeltServo,-1))
                            .waitSeconds(3.75)
                            .build()
            );
        }

    }
    public static class LaunchLeft implements Action {
        CRServo LeftBeltServo;
        double Balls;
        double LeftRuntime;
        double Dir;
        ElapsedTime LaunchTimer;

        public LaunchLeft(CRServo LS,double balls,double dir) {
            this.Balls = balls;
            this.LeftBeltServo = LS;
            this.Dir = dir;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            if (LaunchTimer == null) {
                LaunchTimer = new ElapsedTime();
                LeftRuntime = Balls;
                while (LaunchTimer.seconds() < LeftRuntime){
                    LeftBeltServo.setPower(-1*Dir);
                }
            }
            LeftBeltServo.setPower(0);

            return false;
        }
    }
    public static class StartBelt implements Action {
        CRServo LeftBeltServo;
        double Dir;
        ElapsedTime LaunchTimer;

        public StartBelt(CRServo LS,double dir) {
            this.LeftBeltServo = LS;
            this.Dir = dir;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            if (LaunchTimer == null) {
                LaunchTimer = new ElapsedTime();
                LeftBeltServo.setPower(-1*Dir);
            }

            return false;
        }
    }
    public static class LaunchRight implements Action {
        CRServo RightBeltServo;
        double Balls;
        double RightRuntime;
        ElapsedTime LaunchTimer;

        public LaunchRight(CRServo RS,double time) {
            this.Balls = time;
            this.RightBeltServo = RS;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            if (LaunchTimer == null) {
                LaunchTimer = new ElapsedTime();
                RightRuntime = Balls;
                while (LaunchTimer.seconds() < RightRuntime){
                    RightBeltServo.setPower(1);
                }
            }
            RightBeltServo.setPower(0);

            return false;
        }
    }

    public static class RightIntakeStart implements Action {

        CRServo Left;
        CRServo Right;
        CRServo Belt;
        double dir;

        ElapsedTime LaunchTimer;

        public RightIntakeStart(CRServo L,CRServo R, CRServo B,double dir) {
            this.Left =L;
            this.Right = R;
            this.Belt = B;
            this.dir = dir;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            if (LaunchTimer == null) {
                LaunchTimer = new ElapsedTime();
                Left.setPower(1*dir);
                Right.setPower(-1*dir);
                Belt.setPower(-1*dir);
            }

            return false;
        }
    }

    public static class LeftIntakeStart implements Action {

        CRServo Left;
        CRServo Right;
        CRServo Belt;
        double dir;

        ElapsedTime LaunchTimer;

        public LeftIntakeStart(CRServo L,CRServo R, CRServo B,double dir) {
            this.Left =L;
            this.Right = R;
            this.Belt = B;
            this.dir = dir;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            if (LaunchTimer == null) {
                LaunchTimer = new ElapsedTime();
                Left.setPower(-1*dir);
                Right.setPower(-1*dir);
                Belt.setPower(1*dir);
            }

            return false;
        }
    }

    public static class TopRightIntake implements Action {

        Servo Top;
        double pos;

        ElapsedTime LaunchTimer;

        public TopRightIntake(Servo T,double pos) {
            this.Top = T;
            this.pos = pos;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            if (LaunchTimer == null) {
                LaunchTimer = new ElapsedTime();
                if (pos == 1){
                    Top.setPosition(.472);
                } else if (pos == 2) {
                    Top.setPosition(.485);
                } else if (pos == 3) {
                    Top.setPosition(.51);
                }
            }

            return false;
        }
    }

    public static class TopLeftIntake implements Action {

        Servo Top;
        double pos;

        ElapsedTime LaunchTimer;

        public TopLeftIntake(Servo T,double pos) {
            this.Top = T;
            this.pos = pos;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            if (LaunchTimer == null) {
                LaunchTimer = new ElapsedTime();
                if (pos == 1){
                    Top.setPosition(.51);
                } else if (pos == 2) {
                    Top.setPosition(.5);
                } else if (pos == 3) {
                    Top.setPosition(.475);
                }
            }

            return false;
        }
    }

    public static class IntakeStop implements Action {

        CRServo Left;
        CRServo Right;
        CRServo Belt;

        ElapsedTime LaunchTimer;

        public IntakeStop(CRServo L,CRServo R, CRServo B) {
            this.Left =L;
            this.Right = R;
            this.Belt = B;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            if (LaunchTimer == null) {
                LaunchTimer = new ElapsedTime();
                Left.setPower(0);
                Right.setPower(0);
                Belt.setPower(0);
            }

            return false;
        }
    }

    public class PrimeLaunch implements Action {

        DcMotor LeftLauncher;
        DcMotor RightLauncher;

        double LeftPower;
        double RightPower;
        double PrimeTime;

        double voltMult; // passed in

        // If you still want left/right trim, make it explicit:
        double leftTrim = 0.00;
        double rightTrim = 0.00;

        ElapsedTime LaunchTimer;

        public PrimeLaunch(DcMotor L, DcMotor R, double LP, double RP, double PR, double voltMult) {
            this.LeftLauncher = L;
            this.RightLauncher = R;
            this.LeftPower = LP;
            this.RightPower = RP;
            this.PrimeTime = PR;
            this.voltMult = voltMult;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            if (LaunchTimer == null) {
                LaunchTimer = new ElapsedTime();

                // Final commanded powers (what you're actually sending to the motors)
                double leftOut  = (LeftPower + leftTrim) * voltMult;
                double rightOut = (-RightPower + rightTrim) * voltMult;

                LeftLauncher.setPower(leftOut);
                RightLauncher.setPower(rightOut);

// Save for DS telemetry
                lastLeftLauncherPower = leftOut;
                lastRightLauncherPower = rightOut;

// (optional) dashboard packet too
                telemetryPacket.put("LeftLauncherPower", leftOut);
                telemetryPacket.put("RightLauncherPower", rightOut);
            }

            return LaunchTimer.seconds() < PrimeTime;
        }
    }

    public static class StopLaunch implements Action {
        DcMotor LeftLauncher;
        DcMotor RightLauncher;
        ElapsedTime LaunchTimer;

        public StopLaunch(DcMotor L, DcMotor R) {
            this.LeftLauncher = L;
            this.RightLauncher = R;

        }

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {

            LaunchTimer = new ElapsedTime();
            LeftLauncher.setPower(0);
            RightLauncher.setPower(0);


            return false;
        }
    }

    public static class StrafeLeftSlow implements Action {
        DcMotor leftFront;
        DcMotor rightFront;
        DcMotor leftBack;
        DcMotor rightBack;
        double Speed;
        double Time;
        ElapsedTime LaunchTimer;

        public StrafeLeftSlow(DcMotor FL, DcMotor FR, DcMotor BL,DcMotor BR, double S, double T) {
            this.leftFront = FL;
            this.rightFront = FR;
            this.leftBack = BL;
            this.rightBack = BR;
            this.Speed = S;
            this.Time = T;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            if (LaunchTimer == null) {
                LaunchTimer = new ElapsedTime();
                while (LaunchTimer.seconds() < Time) {
                    leftFront.setPower(-Speed);
                    rightFront.setPower(Speed);
                    leftBack.setPower(Speed);
                    rightBack.setPower(-Speed);
                }
            }
            leftFront.setPower(0);
            rightFront.setPower(0);
            leftBack.setPower(0);
            rightBack.setPower(0);

            return false;
        }
    }
}
