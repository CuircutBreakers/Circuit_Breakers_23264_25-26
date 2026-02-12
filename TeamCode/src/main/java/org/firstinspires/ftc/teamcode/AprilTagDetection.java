package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "AprilTag: HuskyLens Basic", group = "Vision")
public class AprilTagDetection extends LinearOpMode {

    private HuskyLens huskyLens;

    @Override
    public void runOpMode() {

        // Hardware Map name must match your Robot Config name
        huskyLens = hardwareMap.get(HuskyLens.class, "huskylens");

        // Quick comms check
        if (!huskyLens.knock()) {
            telemetry.addLine("HuskyLens NOT responding.");
            telemetry.addLine("Check: cable, power, I2C address, Robot Config name = 'huskylens'");
        } else {
            telemetry.addLine("HuskyLens OK. Press START.");
        }
        telemetry.update();

        // Select AprilTag mode (36h11 family)
        huskyLens.selectAlgorithm(HuskyLens.Algorithm.TAG_RECOGNITION);

        waitForStart();

        while (opModeIsActive()) {

            // Get all detected tags/objects
            HuskyLens.Block[] blocks = huskyLens.blocks();

            telemetry.addData("Detected", blocks.length);

            // Each Block corresponds to one detected item (tag bounding box + ID)
            for (int i = 0; i < blocks.length; i++) {
                HuskyLens.Block b = blocks[i];

                // Common useful fields:
                // b.id = Tag ID, b.x/b.y = center, b.width/b.height = box size, b.left/b.top = upper-left
                telemetry.addData("Tag[" + i + "]",
                        "id=%d  x=%d y=%d  w=%d h=%d",
                        b.id, b.x, b.y, b.width, b.height
                );
            }

            telemetry.update();
        }
    }
}
