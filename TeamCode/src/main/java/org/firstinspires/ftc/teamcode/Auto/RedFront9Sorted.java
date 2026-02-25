package org.firstinspires.ftc.teamcode.Auto;


import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.AngularVelConstraint;
import com.acmerobotics.roadrunner.MecanumKinematics;
import com.acmerobotics.roadrunner.MinVelConstraint;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.VelConstraint;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.MecanumDrive;

import java.util.Arrays;

@Autonomous
public class RedFront9Sorted extends LinearOpMode {

    HuskyLens Camera;





    // Over-voltage reduction (tune)
    // Voltage compensation (tune)
// Target band ~13.8V (typical "good" under load)

    @Override
    public void runOpMode() throws InterruptedException {

        //region init
        MecanumDrive.Params PARAMS = new MecanumDrive.Params();

        MecanumKinematics kinematics = new MecanumKinematics(
                PARAMS.inPerTick * PARAMS.trackWidthTicks, PARAMS.inPerTick / PARAMS.lateralInPerTick);

        MinVelConstraint IntakeSpeed = new MinVelConstraint(Arrays.asList(kinematics.new WheelVelConstraint(19), new AngularVelConstraint(PARAMS.maxAngVel)));

        VelConstraint IntakeSpeedSlow = new MinVelConstraint(Arrays.asList(kinematics.new WheelVelConstraint(17), new AngularVelConstraint(PARAMS.maxAngVel)));

        MinVelConstraint IntakeSpeedFast = new MinVelConstraint(Arrays.asList(kinematics.new WheelVelConstraint(30), new AngularVelConstraint(PARAMS.maxAngVel)));


        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(52,-53,Math.toRadians(-51)));
        DcMotorEx LeftLauncher = hardwareMap.get(DcMotorEx.class, "LeftLauncher");
        DcMotorEx RightLauncher = hardwareMap.get(DcMotorEx.class, "RightLauncher");

        CRServo RightIntakeLeftServo = hardwareMap.get(CRServo.class, "RightIntakeLeftServo");
        CRServo RightIntakeRightServo = hardwareMap.get(CRServo.class, "RightIntakeRightServo");
        Servo LeftTopServo = hardwareMap.get(Servo.class, "LeftIntakeTopServo");
        CRServo LeftIntakeLeftServo = hardwareMap.get(CRServo.class, "LeftIntakeLeftServo");
        CRServo LeftIntakeRightServo = hardwareMap.get(CRServo.class, "LeftIntakeRightServo");
        Servo RightTopServo = hardwareMap.get(Servo.class, "RightIntakeTopServo");
        DcMotor LeftBeltMotor = hardwareMap.get(DcMotor.class, "leftBeltMotor");
        DcMotor RightBeltMotor = hardwareMap.get(DcMotor.class, "rightBeltMotor");

        RightBeltMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        LeftLauncher.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RightLauncher.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        LeftLauncher.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RightLauncher.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        LeftLauncher.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        RightLauncher.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        Camera = hardwareMap.get(HuskyLens.class, "huskylens");
        Camera.selectAlgorithm(HuskyLens.Algorithm.TAG_RECOGNITION);

        Servo RightLight    = hardwareMap.get(Servo.class, "RightLight");
        Servo LeftLight     = hardwareMap.get(Servo.class, "LeftLight");

        //Intake pushers Init
        RightTopServo.setPosition(.485);
        LeftTopServo.setPosition(.5);
        //Ball position Init
        LeftBeltMotor.setPower(-1);
        RightBeltMotor.setPower(-1);
        sleep(200);
        LeftBeltMotor.setPower(0);
        RightBeltMotor.setPower(0);
        RightLight.setPosition(.5);
        LeftLight.setPosition(.72);
        sleep(1000);
        RightLight.setPosition(.28);
        LeftLight.setPosition(.28);
        //endregion

        waitForStart();
//region stage 1
        TagResult tag = new TagResult();

        Pose2d startPose = new Pose2d(52, -53, Math.toRadians(-51));

        Action Move1 = drive.actionBuilder(startPose)
                .strafeToLinearHeading(new Vector2d(25, -25), Math.toRadians(10))
                .turnTo(Math.toRadians(-46))
                .build();




        Actions.runBlocking(
                new ParallelAction(
                        Move1,
                        new PrimeLaunchers(LeftLauncher, RightLauncher, 1700, 1700, 1),
                        new DetectAprilTag(Camera, tag, 2.5)
                        

                )

        );


//endregion
        int detectedId = tag.id;  // <-- THIS is your result

        telemetry.addData("Final Tag ID", detectedId);
        telemetry.update();
        if (detectedId == 1){

            drive.updatePoseEstimate();
            Actions.runBlocking(
                    drive.actionBuilder(drive.localizer.getPose())
                            .afterTime(0.5,new StartBelt(LeftBeltMotor,1))
                            .afterTime(0,new StartBelt(RightBeltMotor,1))
                            .waitSeconds(1)
                            .afterTime(.25,new StopLaunch(LeftLauncher,RightLauncher))
                            .afterTime(0,new TopRightIntake(RightTopServo,3))
                            .afterTime(0,new RightIntakeStart(RightIntakeLeftServo,RightIntakeRightServo,RightBeltMotor,1))
                            .strafeToLinearHeading(new Vector2d(-8, -28), Math.toRadians(0))
                            //Start intaking ball 1
                            .strafeToLinearHeading(new Vector2d(-8,-39.5),Math.toRadians(0),IntakeSpeedSlow)
                            .afterTime(1.5,new TopRightIntake(RightTopServo,1))
                            .afterTime(2,new TopRightIntake(RightTopServo,3))
                            .afterDisp(0,new LeftIntakeStart(LeftIntakeLeftServo,LeftIntakeRightServo,LeftBeltMotor,1))
                            .splineToLinearHeading(new Pose2d(-8, -37, Math.toRadians(0)),   Math.toRadians(90))
                            .afterDisp(0.0,new TopRightIntake(RightTopServo, 1))
                            .setTangent(Math.toRadians(90))
                            .splineToLinearHeading(new Pose2d(-7, -34, Math.toRadians(90)),  Math.toRadians(90))
                            .afterDisp(0.0,new TopLeftIntake(LeftTopServo, 3))
// lineup for ball 3
                            .setTangent(Math.toRadians(90))
                            .splineToLinearHeading(new Pose2d(-8, -37, Math.toRadians(180)), Math.toRadians(180))
                            .afterDisp(0, new TopRightIntake(RightTopServo, 3))
                            //Start intaking ball 3
                            .afterTime(0,new TopRightIntake(RightTopServo,1))
                            .afterTime(.5,new TopRightIntake(RightTopServo,3))
                            .strafeToLinearHeading(new Vector2d(-8,-60),Math.toRadians(180))
                            .afterDisp(0,new IntakeStop(RightIntakeLeftServo,RightIntakeRightServo,RightBeltMotor))

                            .build()
            );
            drive.updatePoseEstimate();
            Action move4 = drive.actionBuilder(drive.localizer.getPose())
                    .afterDisp(3,new TopLeftIntake(LeftTopServo, 1))
                    .afterDisp(7,new TopLeftIntake(LeftTopServo, 3))
                    .strafeToLinearHeading(new Vector2d(0,-25),Math.toRadians(-37))
                    .afterTime(0,new IntakeStop(LeftIntakeLeftServo,LeftIntakeRightServo,LeftBeltMotor))
                    .strafeToLinearHeading(new Vector2d(25, -25), Math.toRadians(-34))
                    .waitSeconds(1.25)
                    .stopAndAdd(new StopLaunch(LeftLauncher,RightLauncher))
                    .stopAndAdd(new RightIntakeStart(RightIntakeLeftServo,RightIntakeRightServo,RightBeltMotor,1))
                    .stopAndAdd(new TopRightIntake(RightTopServo,3))
                    .strafeToLinearHeading(new Vector2d(16,-28),Math.toRadians(0))
                    .build();

            Actions.runBlocking(new ParallelAction(
                    move4,
                    new PrimeLaunchers(LeftLauncher,RightLauncher,1700,1650,1),
                    new StartBeltDelayed(RightBeltMotor,1,2.25),
                    new StartBeltDelayed(RightBeltMotor,0,2.55),
                    new StartBeltDelayed(LeftBeltMotor,1,2.25),
                    new StartBeltDelayed(LeftBeltMotor,0,2.65),
                    new StartBeltDelayed(LeftBeltMotor,1,4),
                    new StartBeltDelayed(RightBeltMotor,1,3.75)
            ));
            drive.updatePoseEstimate();
            Actions.runBlocking(
                    drive.actionBuilder(drive.localizer.getPose())
                            .strafeToLinearHeading(new Vector2d(16,-30),Math.toRadians(0))
                            //Start intaking ball 4
                            .strafeToLinearHeading(new Vector2d(16,-39.5),Math.toRadians(0),IntakeSpeed)
                            .stopAndAdd(new TopRightIntake(RightTopServo, 1))
                            .stopAndAdd(new LeftIntakeStart(LeftIntakeLeftServo,LeftIntakeRightServo,LeftBeltMotor,1))
                            //Backup and spin
                            .strafeToLinearHeading(new Vector2d(16,-34),Math.toRadians(0))
                            .strafeToLinearHeading(new Vector2d(16,-33),Math.toRadians(90))
                            .stopAndAdd(new TopRightIntake(RightTopServo, 3))
                            .stopAndAdd(new TopLeftIntake(LeftTopServo, 3))
                            //lineup for ball 6
                            .strafeToLinearHeading(new Vector2d(16,-37),Math.toRadians(180))
                            .stopAndAdd(new IntakeStop(RightIntakeLeftServo,RightIntakeRightServo,RightBeltMotor))
                            //Start intaking ball 6
                            .strafeToLinearHeading(new Vector2d(16,-52),Math.toRadians(180))
                            .afterTime(.75,new TopLeftIntake(LeftTopServo, 1))
                            .afterTime(1.25,new TopLeftIntake(LeftTopServo, 3))
                            .strafeToLinearHeading(new Vector2d(20,-30),Math.toRadians(-120))
                            .build()
            );

            drive.updatePoseEstimate();
            Action move5 = drive.actionBuilder(drive.localizer.getPose())
                    .strafeToLinearHeading(new Vector2d(25, -25), Math.toRadians(-33))
                    .turnTo(Math.toRadians(-30))
                    .waitSeconds(1.5)
                    .strafeToLinearHeading(new Vector2d(3,-40),Math.toRadians(-90))
                    .build();

            Actions.runBlocking(new ParallelAction(
                    move5,
                    new PrimeLaunchers(LeftLauncher,RightLauncher,1650,1625,1),
                    new StartBeltDelayed(RightBeltMotor,1,.25),
                    new StartBeltDelayed(RightBeltMotor,0,.5),
                    new StartBeltDelayed(RightBeltMotor,1,1.33),
                    new StartBeltDelayed(LeftBeltMotor,1,.3)
            ));

        }
        else if (detectedId == 2) {

            drive.updatePoseEstimate();
            Actions.runBlocking(
                    drive.actionBuilder(drive.localizer.getPose())
                            .afterTime(0,new StartBelt(LeftBeltMotor,1))
                            .afterTime(.3,new StartBelt(LeftBeltMotor,0))
                            .afterTime(.9,new StartBelt(RightBeltMotor,1))
                            .afterTime(1.25,new StartBelt(LeftBeltMotor,1))
                            .waitSeconds(1.5)
                            .afterTime(.25,new StopLaunch(LeftLauncher,RightLauncher))
                            .afterTime(0,new TopRightIntake(RightTopServo,3))
                            .afterTime(0,new RightIntakeStart(RightIntakeLeftServo,RightIntakeRightServo,RightBeltMotor,1))
                            .strafeToLinearHeading(new Vector2d(-8, -28), Math.toRadians(0))
                            //Start intaking ball 1
                            .strafeToLinearHeading(new Vector2d(-8,-39.5),Math.toRadians(0),IntakeSpeedSlow)
                            .afterTime(1.5,new TopRightIntake(RightTopServo,1))
                            .afterTime(2,new TopRightIntake(RightTopServo,3))
                            .afterDisp(0,new LeftIntakeStart(LeftIntakeLeftServo,LeftIntakeRightServo,LeftBeltMotor,1))
                            .splineToLinearHeading(new Pose2d(-8, -37, Math.toRadians(0)),   Math.toRadians(90))
                            .afterDisp(0.0,new TopRightIntake(RightTopServo, 1))
                            .setTangent(Math.toRadians(90))
                            .splineToLinearHeading(new Pose2d(-7, -34, Math.toRadians(90)),  Math.toRadians(90))
                            .afterDisp(0.0,new TopLeftIntake(LeftTopServo, 3))
// lineup for ball 3
                            .setTangent(Math.toRadians(90))
                            .splineToLinearHeading(new Pose2d(-8, -37, Math.toRadians(180)), Math.toRadians(180))
                            .afterDisp(0, new TopRightIntake(RightTopServo, 3))
                            //Start intaking ball 3
                            .afterTime(0,new TopRightIntake(RightTopServo,1))
                            .afterTime(.5,new TopRightIntake(RightTopServo,3))
                            .strafeToLinearHeading(new Vector2d(-8,-60),Math.toRadians(180))
                            .afterDisp(0,new IntakeStop(RightIntakeLeftServo,RightIntakeRightServo,RightBeltMotor))

                            .build()
            );
            drive.updatePoseEstimate();
            Action move4 = drive.actionBuilder(drive.localizer.getPose())
                    .afterDisp(3,new TopLeftIntake(LeftTopServo, 1))
                    .afterDisp(7,new TopLeftIntake(LeftTopServo, 3))
                    .strafeToLinearHeading(new Vector2d(0,-25),Math.toRadians(-37))
                    .afterTime(0,new IntakeStop(LeftIntakeLeftServo,LeftIntakeRightServo,LeftBeltMotor))
                    .strafeToLinearHeading(new Vector2d(25, -25), Math.toRadians(-34))
                    .waitSeconds(1.25)
                    .stopAndAdd(new StopLaunch(LeftLauncher,RightLauncher))
                    .stopAndAdd(new RightIntakeStart(RightIntakeLeftServo,RightIntakeRightServo,RightBeltMotor,1))
                    .stopAndAdd(new TopRightIntake(RightTopServo,3))
                    .strafeToLinearHeading(new Vector2d(16,-28),Math.toRadians(0))
                    .build();

            Actions.runBlocking(new ParallelAction(
                    move4,
                    new PrimeLaunchers(LeftLauncher,RightLauncher,1700,1650,1),
                    new StartBeltDelayed(RightBeltMotor,1,2.25),
                    new StartBeltDelayed(RightBeltMotor,0,2.55),
                    new StartBeltDelayed(LeftBeltMotor,1,2.25),
                    new StartBeltDelayed(LeftBeltMotor,0,2.85),
                    new StartBeltDelayed(LeftBeltMotor,1,3.75),
                    new StartBeltDelayed(RightBeltMotor,1,4),
                    new StartBeltDelayed(RightBeltMotor,0,4.25),
                    new StartBeltDelayed(RightBeltMotor,1,4.75)
            ));
            drive.updatePoseEstimate();
            Actions.runBlocking(
                    drive.actionBuilder(drive.localizer.getPose())
                            .strafeToLinearHeading(new Vector2d(16,-30),Math.toRadians(0))
                            //Start intaking ball 4
                            .strafeToLinearHeading(new Vector2d(16,-39.5),Math.toRadians(0),IntakeSpeed)
                            .stopAndAdd(new TopRightIntake(RightTopServo, 1))
                            .stopAndAdd(new LeftIntakeStart(LeftIntakeLeftServo,LeftIntakeRightServo,LeftBeltMotor,1))
                            //Backup and spin
                            .strafeToLinearHeading(new Vector2d(16,-34),Math.toRadians(0))
                            .strafeToLinearHeading(new Vector2d(16,-33),Math.toRadians(90))
                            .stopAndAdd(new TopRightIntake(RightTopServo, 3))
                            .stopAndAdd(new TopLeftIntake(LeftTopServo, 3))
                            //lineup for ball 6
                            .strafeToLinearHeading(new Vector2d(16,-37),Math.toRadians(180))
                            .stopAndAdd(new IntakeStop(RightIntakeLeftServo,RightIntakeRightServo,RightBeltMotor))
                            //Start intaking ball 6
                            .strafeToLinearHeading(new Vector2d(16,-52),Math.toRadians(180))
                            .afterTime(.75,new TopLeftIntake(LeftTopServo, 1))
                            .afterTime(1.25,new TopLeftIntake(LeftTopServo, 3))
                            .strafeToLinearHeading(new Vector2d(20,-30),Math.toRadians(-120))
                            .build()
            );

            drive.updatePoseEstimate();
            Action move5 = drive.actionBuilder(drive.localizer.getPose())
                    .strafeToLinearHeading(new Vector2d(25, -25), Math.toRadians(-30))
                    .turnTo(Math.toRadians(-33))
                    .waitSeconds(1.5)
                    .strafeToLinearHeading(new Vector2d(3,-40),Math.toRadians(-90))
                    .build();

            Actions.runBlocking(new ParallelAction(
                    move5,
                    new PrimeLaunchers(LeftLauncher,RightLauncher,1650,1625,1),
                    new StartBeltDelayed(RightBeltMotor,1,.25),
                    new StartBeltDelayed(RightBeltMotor,0,1),
                    new StartBeltDelayed(RightBeltMotor,1,1.75),
                    new StartBeltDelayed(LeftBeltMotor,1,.5),
                    new StartBeltDelayed(LeftBeltMotor,0,1),
                    new StartBeltDelayed(LeftBeltMotor,1,1.5)
            ));

        }
        else if (detectedId == 3) {

            drive.updatePoseEstimate();
            Actions.runBlocking(
                    drive.actionBuilder(drive.localizer.getPose())
                            .afterTime(0,new StartBelt(LeftBeltMotor,1))
                            .afterTime(1,new StartBelt(RightBeltMotor,1))
                            .waitSeconds(1.5)
                            .afterTime(.25,new StopLaunch(LeftLauncher,RightLauncher))
                            .afterTime(0,new TopRightIntake(RightTopServo,3))
                            .afterTime(0,new RightIntakeStart(RightIntakeLeftServo,RightIntakeRightServo,RightBeltMotor,1))
                            .strafeToLinearHeading(new Vector2d(-8, -28), Math.toRadians(0))
                            //Start intaking ball 1
                            .strafeToLinearHeading(new Vector2d(-8,-33.5),Math.toRadians(0),IntakeSpeed)
                            .afterDisp(0,new LeftIntakeStart(LeftIntakeLeftServo,LeftIntakeRightServo,LeftBeltMotor,1))
                            .splineToLinearHeading(new Pose2d(-8, -32, Math.toRadians(0)),   Math.toRadians(90))
                            .afterDisp(0.0,new TopRightIntake(RightTopServo, 1))
                            .setTangent(Math.toRadians(90))
                            .splineToLinearHeading(new Pose2d(-7, -29, Math.toRadians(90)),  Math.toRadians(90))
                            .afterDisp(0.0,new TopLeftIntake(LeftTopServo, 3))
// lineup for ball 3
                            .setTangent(Math.toRadians(90))
                            .splineToLinearHeading(new Pose2d(-8, -32, Math.toRadians(180)), Math.toRadians(180))
                            .afterDisp(0, new TopRightIntake(RightTopServo, 3))
                            //Start intaking ball 3
                            .afterTime(.3, new TopRightIntake(RightTopServo,1))
                            .afterTime(.6,new TopRightIntake(RightTopServo, 3))
                            .strafeToLinearHeading(new Vector2d(-8,-63),Math.toRadians(180),IntakeSpeedFast)
                            .build()
            );
            drive.updatePoseEstimate();
            Action move4 = drive.actionBuilder(drive.localizer.getPose())
                    .afterDisp(3,new TopLeftIntake(LeftTopServo, 1))
                    .afterDisp(7,new TopLeftIntake(LeftTopServo, 3))
                    .afterDisp(5,new PrimeLaunchers(LeftLauncher,RightLauncher,1725,1650,1))
                    .strafeToLinearHeading(new Vector2d(0,-25),Math.toRadians(-37))
                    .afterTime(0,new IntakeStop(LeftIntakeLeftServo,LeftIntakeRightServo,LeftBeltMotor))
                    .strafeToLinearHeading(new Vector2d(25, -25), Math.toRadians(-34))
                    .waitSeconds(1.25)
                    .stopAndAdd(new StopLaunch(LeftLauncher,RightLauncher))
                    .stopAndAdd(new RightIntakeStart(RightIntakeLeftServo,RightIntakeRightServo,RightBeltMotor,1))
                    .stopAndAdd(new TopRightIntake(RightTopServo,3))
                    .strafeToLinearHeading(new Vector2d(16,-28),Math.toRadians(0))
                    .build();

            Actions.runBlocking(new ParallelAction(
                    move4,
                    new StartBeltDelayed(RightBeltMotor,1,2.25),
                    new StartBeltDelayed(RightBeltMotor,0,2.75),
                    new StartBeltDelayed(LeftBeltMotor,1,2.25),
                    new StartBeltDelayed(LeftBeltMotor,0,2.75),
                    new StartBeltDelayed(LeftBeltMotor,1,3.75),
                    new StartBeltDelayed(LeftBeltMotor,0,4),
                    new StartBeltDelayed(LeftBeltMotor,1,4.5),
                    new StartBeltDelayed(RightBeltMotor,1,3.75)
            ));
            drive.updatePoseEstimate();
            Actions.runBlocking(
                    drive.actionBuilder(drive.localizer.getPose())
                            .strafeToLinearHeading(new Vector2d(16,-30),Math.toRadians(0))
                            //Start intaking ball 4
                            .strafeToLinearHeading(new Vector2d(16,-38.5),Math.toRadians(0),IntakeSpeed)
                            .stopAndAdd(new TopRightIntake(RightTopServo, 1))
                            .stopAndAdd(new LeftIntakeStart(LeftIntakeLeftServo,LeftIntakeRightServo,LeftBeltMotor,1))
                            //Backup and spin
                            .strafeToLinearHeading(new Vector2d(16,-34),Math.toRadians(0))
                            .strafeToLinearHeading(new Vector2d(16,-33),Math.toRadians(90))
                            .stopAndAdd(new TopRightIntake(RightTopServo, 3))
                            .stopAndAdd(new TopLeftIntake(LeftTopServo, 3))
                            //lineup for ball 6
                            .strafeToLinearHeading(new Vector2d(16,-37),Math.toRadians(180))
                            .stopAndAdd(new IntakeStop(RightIntakeLeftServo,RightIntakeRightServo,RightBeltMotor))
                            //Start intaking ball 6
                            .strafeToLinearHeading(new Vector2d(16,-52),Math.toRadians(180))
                            .afterTime(.75,new TopLeftIntake(LeftTopServo, 1))
                            .afterTime(1.25,new TopLeftIntake(LeftTopServo, 3))
                            .strafeToLinearHeading(new Vector2d(20,-30),Math.toRadians(-120))
                            .build()
            );

            drive.updatePoseEstimate();
            Action move5 = drive.actionBuilder(drive.localizer.getPose())
                    .strafeToLinearHeading(new Vector2d(25, -25), Math.toRadians(-26))
                    .waitSeconds(1.5)
                    .turnTo(Math.toRadians(-33))
                    .waitSeconds(.25)
                    .strafeToLinearHeading(new Vector2d(3,-40),Math.toRadians(-90))
                    .build();

            Actions.runBlocking(new ParallelAction(
                    move5,
                    new PrimeLaunchers(LeftLauncher,RightLauncher,1650,1575,1),
                    new StartBeltDelayed(LeftBeltMotor,1,.25),
                    new StartBeltDelayed(LeftBeltMotor,0,.5),
                    new StartBeltDelayed(LeftBeltMotor,1,1.6),
                    new StartBeltDelayed(RightBeltMotor,1,.3)
            ));

        }

    }


    public static class StartBeltDelayed implements Action {
        DcMotor BeltMotor;
        double Dir;
        double Time;
        ElapsedTime LaunchTimer;

        public StartBeltDelayed(DcMotor Motor,double power, double Time) {
            this.BeltMotor = Motor;
            this.Dir = power;
            this.Time = Time;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            if (LaunchTimer == null) {
                LaunchTimer = new ElapsedTime();
            }
            if (LaunchTimer.seconds() >= Time) {
                BeltMotor.setPower(Dir);
                return false;
            }

            return true;
        }
    }
    public static class StartBelt implements Action {
        DcMotor BeltMotor;
        double Dir;
        ElapsedTime LaunchTimer;

        public StartBelt(DcMotor Motor,double power) {
            this.BeltMotor = Motor;
            this.Dir = power;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            if (LaunchTimer == null) {
                LaunchTimer = new ElapsedTime();
            }

                BeltMotor.setPower(Dir);
                return false;


        }
    }


    public static class RightIntakeStart implements Action {

        CRServo Left;
        CRServo Right;
        DcMotor Belt;
        double dir;

        ElapsedTime LaunchTimer;

        public RightIntakeStart(CRServo L,CRServo R, DcMotor B,double dir) {
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
        DcMotor Belt;
        double dir;

        ElapsedTime LaunchTimer;

        public LeftIntakeStart(CRServo L,CRServo R, DcMotor B,double dir) {
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
                Belt.setPower(-1*dir);
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
        DcMotor Belt;

        ElapsedTime LaunchTimer;

        public IntakeStop(CRServo L,CRServo R, DcMotor B) {
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

    public static class StopLaunch implements Action {

        private final DcMotorEx leftLauncher;
        private final DcMotorEx rightLauncher;

        private boolean initialized = false;

        public StopLaunch(DcMotorEx left, DcMotorEx right) {
            this.leftLauncher = left;
            this.rightLauncher = right;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {

            if (!initialized) {
                initialized = true;

                leftLauncher.setVelocity(0);
                rightLauncher.setVelocity(0);
            }

            return false; // instant action
        }
    }


    public static class PrimeLaunchers implements Action {

        private final DcMotorEx left;
        private final DcMotorEx right;
        private final double targetVelLeft;
        private final double targetVelRight;
        private final int direction;

        private boolean initialized = false;

        public PrimeLaunchers(DcMotorEx left, DcMotorEx right,
                              double targetVelLeft,
                              double targetVelRight,
                              int direction) {
            this.left = left;
            this.right = right;
            this.targetVelLeft = targetVelLeft;
            this.targetVelRight = targetVelRight;
            this.direction = direction;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (!initialized) {
                initialized = true;

                left.setVelocity((targetVelLeft + 150) * direction);
                right.setVelocity(-(targetVelRight + 150) * direction);
            }

            return false; // end immediately
        }
    }


    public static class TagResult {
        public volatile int id = 0;          // 0 = not found yet
        public volatile boolean found = false;
    }

    public static class DetectAprilTag implements Action {

        private final HuskyLens camera;
        private final TagResult out;
        private final double timeoutSec;

        private ElapsedTime timer;

        public DetectAprilTag(HuskyLens camera, TagResult out, double timeoutSec) {
            this.camera = camera;
            this.out = out;
            this.timeoutSec = timeoutSec;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {

            if (timer == null) timer = new ElapsedTime();

            HuskyLens.Block[] blocks = camera.blocks();

            for (HuskyLens.Block b : blocks) {

                // Only accept IDs 1–3
                if (b.id >= 1 && b.id <= 3) {
                    out.id = b.id;
                    out.found = true;

                    packet.put("tag/found", true);
                    packet.put("tag/id", out.id);

                    return false; // ✅ stop checking permanently
                }
            }

            // Optional timeout fallback
            if (timer.seconds() >= timeoutSec) {
                out.id = 1;          // default fallback
                out.found = false;

                packet.put("tag/timeout", true);
                return false;
            }

            packet.put("tag/found", false);
            return true; // keep checking
        }
    }




}