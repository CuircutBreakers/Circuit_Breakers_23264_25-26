package org.firstinspires.ftc.teamcode.Teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "Light Test", group = "TeleOp")
public class LightTest extends LinearOpMode {

    private Servo LeftLight;

    // Preset servo positions (0.0 to 1.0)
    private double[] positions = {.28,.5,.72,1};

    private int currentIndex = 0;
    private boolean lastAState = false;

    @Override
    public void runOpMode() {

        LeftLight = hardwareMap.get(Servo.class, "LeftLight");

        waitForStart();

        while (opModeIsActive()) {

            boolean currentAState = gamepad1.a;

            // Detect button press (not hold)
            if (currentAState && !lastAState) {

                // Move to next position
                currentIndex++;

                // Wrap back to 0 if at end
                if (currentIndex >= positions.length) {
                    currentIndex = 0;
                }

                LeftLight.setPosition(positions[currentIndex]);
            }

            lastAState = currentAState;

            telemetry.addData("Stage", currentIndex);
            telemetry.addData("Light", positions[currentIndex]);
            telemetry.update();
        }
    }
}