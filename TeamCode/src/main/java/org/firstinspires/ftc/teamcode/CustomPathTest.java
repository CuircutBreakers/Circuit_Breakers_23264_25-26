package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

public final class CustomPathTest extends LinearOpMode {

    @Override

    public void runOpMode() {


        // We want to start the bot at x: 10, y: -8, heading: 90 degrees
        Pose2d startPose = new Pose2d(0, 0, Math.toRadians(0));
        Pose2d traj1end = new Pose2d(30, 30, Math.toRadians(90));
        Pose2d traj2end = new Pose2d(-30, -30, Math.toRadians(180));

        TankDrive.MecanumDrive drive = new TankDrive.MecanumDrive(hardwareMap,startPose);


        Actions.runBlocking(
                 drive.actionBuilder(startPose)
                .splineTo(new Vector2d(30, 30), Math.toRadians(90))
                .splineTo(new Vector2d(-30, -30), Math.toRadians(180))
                .splineTo(new Vector2d(0, 0), Math.toRadians(0))
                .build());
        Actions.runBlocking(
                 drive.actionBuilder(traj1end)

                .build());
        Actions.runBlocking(
                drive.actionBuilder(traj2end)

                        .build());

    }
    }


