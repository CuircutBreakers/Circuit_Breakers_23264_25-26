package org.firstinspires.ftc.teamcode.Auto;


import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
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

import org.firstinspires.ftc.teamcode.TankDrive;

@Autonomous
public class RedGoalGate6Patterned extends LinearOpMode {

    HuskyLens Camera;





    // Over-voltage reduction (tune)
    // Voltage compensation (tune)
// Target band ~13.8V (typical "good" under load)

    @Override
    public void runOpMode() throws InterruptedException {

        TankDrive.MecanumDrive drive = new TankDrive.MecanumDrive(hardwareMap, new Pose2d(52,-53,Math.toRadians(-51)));
        DcMotorEx LeftLauncher = hardwareMap.get(DcMotorEx.class, "LeftLauncher");
        DcMotorEx RightLauncher = hardwareMap.get(DcMotorEx.class, "RightLauncher");
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

        //Intake pushers Init
        RightTopServo.setPosition(.485);
        LeftTopServo.setPosition(.5);
        //Ball position Init
        LeftBeltMotor.setPower(-1);
        RightBeltMotor.setPower(-1);
        sleep(200);
        LeftBeltMotor.setPower(0);
        RightBeltMotor.setPower(0);

        waitForStart();



        waitForStart();

        TagResult tag = new TagResult();

        Pose2d startPose = new Pose2d(52, -53, Math.toRadians(-51));

// Optional but recommended

        Action Move1 = drive.actionBuilder(startPose)
                .strafeToLinearHeading(new Vector2d(30,-30), Math.toRadians(-43))
                .build();



        Actions.runBlocking(
                new ParallelAction(
                        Move1,
                        new PrimeLaunchers(LeftLauncher, RightLauncher, 1650, 1550, 1),
                        new StartBeltDelayed(LeftBeltMotor,1,.9),
                        new StartBeltDelayed(RightBeltMotor, 1, 1)

                )

        );Pose2d start2 = drive.localizer.getPose();
        Pose2d end2   = new Pose2d(15, -20, Math.toRadians(0));

        double dx2 = end2.position.x - start2.position.x;
        double dy2 = end2.position.y - start2.position.y;
        double travelAngle2 = Math.atan2(dy2, dx2);

        Action move2 = drive.actionBuilder(start2)
                .setTangent(travelAngle2)
                .splineToLinearHeading(end2, travelAngle2)
                .build();

        Actions.runBlocking(move2); // no ParallelAction wrapper


        Pose2d start3 = drive.localizer.getPose();
        Pose2d end3   = new Pose2d(-7, -25, Math.toRadians(0));

        double dx3 = end3.position.x - start3.position.x;
        double dy3 = end3.position.y - start3.position.y;
        double travelAngle3 = Math.atan2(dy3, dx3);

        Action move3 = drive.actionBuilder(start3)
                .setTangent(travelAngle3)
                .splineToLinearHeading(end3, travelAngle3)
                .build();

        Actions.runBlocking(new ParallelAction(
                move3,
                new DetectAprilTag(Camera, tag, 1.25),
                new TopRightIntake(RightTopServo, 3),
                new RightIntakeStart(RightIntakeLeftServo,RightIntakeRightServo,RightBeltMotor,1)

        ));


        int detectedId = tag.id;  // <-- THIS is your result

        telemetry.addData("Final Tag ID", detectedId);
        telemetry.update();
        if (detectedId == 1){

            drive.updatePoseEstimate();
        Actions.runBlocking(
                drive.actionBuilder(drive.localizer.getPose())
                        .stopAndAdd(new StopLaunch(LeftLauncher,RightLauncher))
                        .strafeToLinearHeading(new Vector2d(-7,-30),Math.toRadians(0))
                        .stopAndAdd(new StrafeSlow(frontLeft,frontRight,backLeft,backRight,.3,.85,-1))
                        .stopAndAdd(new TopRightIntake(RightTopServo,1))
                        .waitSeconds(.4)
                        .stopAndAdd(new TopRightIntake(RightTopServo,3))
                        .stopAndAdd(new StrafeSlow(frontLeft,frontRight,backLeft,backRight,.3,.85,-1))
                        .stopAndAdd(new StrafeSlow(frontLeft,frontRight,backLeft,backRight,0,.25,-1))
                        .stopAndAdd(new StrafeSlow(frontLeft,frontRight,backLeft,backRight,1,.65,1))
                        .stopAndAdd(new TopRightIntake(RightTopServo, 1))
                        .stopAndAdd(new TopLeftIntake(LeftTopServo, 3))
                        .build()
        );
            drive.updatePoseEstimate();
            Actions.runBlocking(
                    drive.actionBuilder(drive.localizer.getPose())
                            .stopAndAdd(new LeftIntakeStart(LeftIntakeLeftServo,LeftIntakeRightServo,LeftBeltMotor,-1))
                            .strafeToLinearHeading(new Vector2d(-6.5,-36),Math.toRadians(180))
                            .stopAndAdd(new IntakeStop(RightIntakeLeftServo,RightIntakeRightServo,RightBeltMotor))
                            .stopAndAdd(new TopRightIntake(RightTopServo, 3))
                            .stopAndAdd(new TopLeftIntake(LeftTopServo, 3))
                            .stopAndAdd(new RightIntakeStart(RightIntakeLeftServo,RightIntakeRightServo,RightBeltMotor,-1))
                            .stopAndAdd(new StartBelt(RightBeltMotor,0))
                            .strafeToLinearHeading(new Vector2d(-4.5,-56),Math.toRadians(180))
                            .stopAndAdd(new TopLeftIntake(LeftTopServo, 1))
                            .waitSeconds(.75)
                            .stopAndAdd(new TopLeftIntake(LeftTopServo, 1))
                            .stopAndAdd(new LeftIntakeStart(LeftIntakeLeftServo,LeftIntakeRightServo,LeftBeltMotor,-1))
                            .stopAndAdd(new StartBelt(LeftBeltMotor,0))

                            .build()
            );
            drive.updatePoseEstimate();
            Action move4 = drive.actionBuilder(drive.localizer.getPose())
                    .strafeToLinearHeading(new Vector2d(0,-25),Math.toRadians(-40))
                    .strafeToLinearHeading(new Vector2d(27,-24),Math.toRadians(-36))
                    .waitSeconds(.35)
                    .turnTo(Math.toRadians(-47))
                    .waitSeconds(.25)
                    .strafeToLinearHeading(new Vector2d(15,-30),Math.toRadians(0))
                    .build();

            Actions.runBlocking(new ParallelAction(
                    move4,
                    new PrimeLaunchers(LeftLauncher,RightLauncher,1700,1700,1),
                    new StartBeltDelayed(RightBeltMotor,1,2.5),
                    new StartBeltDelayed(RightBeltMotor,0,2.75),
                    new StartBeltDelayed(RightBeltMotor,1,3),
                    new StartBeltDelayed(LeftBeltMotor,1,3)
            ));

            drive.updatePoseEstimate();
            Actions.runBlocking(
                    drive.actionBuilder(drive.localizer.getPose())
                            .strafeToLinearHeading(new Vector2d(15,-25),Math.toRadians(0))
                            .stopAndAdd(new LeftIntakeStart(LeftIntakeLeftServo,LeftIntakeRightServo,LeftBeltMotor,1))
                            .stopAndAdd(new RightIntakeStart(RightIntakeLeftServo,RightIntakeRightServo,RightBeltMotor,1))
                            .build()
            );

            drive.updatePoseEstimate();
            Actions.runBlocking(
                    drive.actionBuilder(drive.localizer.getPose())
                            .stopAndAdd(new RightIntakeStart(RightIntakeLeftServo,RightIntakeRightServo,RightBeltMotor,1))
                            .strafeToLinearHeading(new Vector2d(15,-30),Math.toRadians(0))
                            .stopAndAdd(new StrafeSlow(frontLeft,frontRight,backLeft,backRight,.35,.75,-1))
                            .stopAndAdd(new StrafeSlow(frontLeft,frontRight,backLeft,backRight,0,.25,-1))
                            .stopAndAdd(new TopRightIntake(RightTopServo, 1))
                            .build()
            );
            drive.updatePoseEstimate();
            Actions.runBlocking(
                    drive.actionBuilder(drive.localizer.getPose())
                            .strafeToLinearHeading(new Vector2d(15,-35),Math.toRadians(0))
                            .stopAndAdd(new TopRightIntake(RightTopServo, 3))
                            .stopAndAdd(new StrafeSlow(frontLeft,frontRight,backLeft,backRight,.35,.75,-1))
                            .stopAndAdd(new StrafeSlow(frontLeft,frontRight,backLeft,backRight,0,.25,-1))
                            .stopAndAdd(new TopRightIntake(RightTopServo, 1))
                            .stopAndAdd(new StrafeSlow(frontLeft,frontRight,backLeft,backRight,1,.25,1))
                            .stopAndAdd(new LeftIntakeStart(LeftIntakeLeftServo,LeftIntakeRightServo,LeftBeltMotor,1))
                            .build()
            );
            drive.updatePoseEstimate();
            Actions.runBlocking(
                    drive.actionBuilder(drive.localizer.getPose())
                            .strafeToLinearHeading(new Vector2d(15,-37),Math.toRadians(180))
                            .stopAndAdd(new TopRightIntake(RightTopServo, 3))
                            .stopAndAdd(new TopLeftIntake(LeftTopServo, 3))
                            .stopAndAdd(new RightIntakeStart(RightIntakeLeftServo,RightIntakeRightServo,RightBeltMotor,-1))
                            .stopAndAdd(new StartBelt(RightBeltMotor,0))
                            .stopAndAdd(new StrafeSlow(frontLeft,frontRight,backLeft,backRight,.75,.5,1))
                            .stopAndAdd(new StrafeSlow(frontLeft,frontRight,backLeft,backRight,0,.25,1))
                            .stopAndAdd(new TopLeftIntake(LeftTopServo, 1))
                            .stopAndAdd(new PrimeLaunchers(LeftLauncher,RightLauncher,1800,1800,1))
                            .stopAndAdd(new TopLeftIntake(LeftTopServo, 3))
                            .stopAndAdd(new LeftIntakeStart(LeftIntakeLeftServo,LeftIntakeRightServo,LeftBeltMotor,-1))
                            .stopAndAdd(new StartBelt(LeftBeltMotor,.125))
                            .strafeToLinearHeading(new Vector2d(25,-25),Math.toRadians(-47))
                            .build()
            );
            drive.updatePoseEstimate();

            Action move5 = drive.actionBuilder(drive.localizer.getPose())
                    .waitSeconds(.075)
                    .turnTo(Math.toRadians(-44))
                    .waitSeconds(1)
                    .strafeToLinearHeading(new Vector2d(60,-15),Math.toRadians(180))
                    .build();



            Actions.runBlocking(new ParallelAction(
                    move5,
                    new StartBelt(LeftBeltMotor,1),
                    new StartBeltDelayed(RightBeltMotor,1,.15)
            ));


        }
        else if (detectedId == 2) {

        }
        else if (detectedId == 3) {

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
                BeltMotor.setPower(Dir);
            }

            return false;
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
                Left.setPower(1*dir);
                Right.setPower(1*dir);
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

    public static class StrafeSlow implements Action {
        private final DcMotor fl, fr, bl, br;
        private final double speed;
        private final double timeSec;
        private final double dir;
        private ElapsedTime timer;

        public StrafeSlow(DcMotor fl, DcMotor fr, DcMotor bl, DcMotor br,
                          double speed, double timeSec, double dir) {
            this.fl = fl; this.fr = fr; this.bl = bl; this.br = br;
            this.speed = speed; this.timeSec = timeSec; this.dir = dir;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (timer == null) timer = new ElapsedTime();

            fl.setPower(-speed * dir);
            fr.setPower( speed * dir);
            bl.setPower( speed * dir);
            br.setPower(-speed * dir);

            if (timer.seconds() >= timeSec) {
                fl.setPower(0);
                fr.setPower(0);
                bl.setPower(0);
                br.setPower(0);
                return false;
            }

            return true;
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
