package org.firstinspires.ftc.teamcode.Teleop;

import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.util.ArrayDeque;

@TeleOp(name="Color Sensor Test", group="Test")
public class ColorSensorTest extends LinearOpMode {

    enum Mode { IDLE, INTAKE, LAUNCH }

    // ---- Distance gate ----
    private final double capCm = 3.5;

    // ---- Tuned hue bands ----
    private final float greenMin = 100f, greenMax = 180f;
    private final float purpleMin = 200f, purpleMax = 280f;

    // ---- Light output positions ----
    private final double GREEN_POS = 0.5;
    private final double PURPLE_POS = 0.72;
    private final double YELLOW_POS = 0.; // separator + pre/post flash
    private final double UNKNOWN_POS = 1.0;

    // ---- Logging ----
    private static final int MAX_LOG = 3;

    // Require a color to be stable this many loops before we consider it "seen"
    private static final int STABLE_LOOPS_REQUIRED = 2;

    // Require a "gap" (UNKNOWN/TOO FAR) this many loops before the next color counts as a new event
    private static final int BREAK_LOOPS_REQUIRED = 6;

    // Require "gap" in LAUNCH mode before we consider a ball actually launched (removal)
    private static final int LAUNCH_GAP_LOOPS_REQUIRED = 2;

    // ---- Flash timing (variable durations) ----
    private static final double YELLOW_SEC = 0.5;
    private static final double COLOR_SEC  = 0.50;

    private final ArrayDeque<String> leftLog = new ArrayDeque<>(MAX_LOG);
    private final ArrayDeque<String> rightLog = new ArrayDeque<>(MAX_LOG);

    // Left side intake event state
    private String leftLastSeen = "NONE";
    private int leftStableCount = 0;
    private int leftBreakCount = BREAK_LOOPS_REQUIRED; // start "ready"
    private boolean leftLoggedThisEvent = false;

    // Right side intake event state
    private String rightLastSeen = "NONE";
    private int rightStableCount = 0;
    private int rightBreakCount = BREAK_LOOPS_REQUIRED; // start "ready"
    private boolean rightLoggedThisEvent = false;

    // Launch removal state (per side)
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

    @Override
    public void runOpMode() {

        // LEFT
        NormalizedColorSensor leftColor = hardwareMap.get(NormalizedColorSensor.class, "LeftColorSensor");
        DistanceSensor leftDistance = hardwareMap.get(DistanceSensor.class, "LeftColorSensor");
        Servo LeftLight = hardwareMap.get(Servo.class, "LeftLight");

        // RIGHT
        NormalizedColorSensor rightColor = hardwareMap.get(NormalizedColorSensor.class, "RightColorSensor");
        DistanceSensor rightDistance = hardwareMap.get(DistanceSensor.class, "RightColorSensor");
        Servo RightLight = hardwareMap.get(Servo.class, "RightLight");

        float[] leftHsv = new float[3];
        float[] rightHsv = new float[3];

        waitForStart();

        while (opModeIsActive()) {

            // -------- Mode selection (gamepad2) --------
            Mode mode = Mode.IDLE;
            if (gamepad2.left_bumper) {
                mode = Mode.LAUNCH; // priority
            } else if (gamepad2.left_stick_y < -0.6) {
                mode = Mode.INTAKE;
            }

            // ---------------- LEFT READ ----------------
            double leftCm = leftDistance.getDistance(DistanceUnit.CM);
            String leftResult = "TOO FAR";

            if (!Double.isNaN(leftCm) && leftCm <= capCm) {

                NormalizedRGBA c = leftColor.getNormalizedColors();

                Color.RGBToHSV(
                        (int)(c.red   * 255),
                        (int)(c.green * 255),
                        (int)(c.blue  * 255),
                        leftHsv
                );

                float h = leftHsv[0];

                if (h >= greenMin && h <= greenMax) leftResult = "GREEN";
                else if (h >= purpleMin && h <= purpleMax) leftResult = "PURPLE";
                else leftResult = "UNKNOWN";

                telemetry.addData("Left Hue", "%.1f", h);
            }

            // ---------------- RIGHT READ ----------------
            double rightCm = rightDistance.getDistance(DistanceUnit.CM);
            String rightResult = "TOO FAR";

            if (!Double.isNaN(rightCm) && rightCm <= capCm) {

                NormalizedRGBA c = rightColor.getNormalizedColors();

                Color.RGBToHSV(
                        (int)(c.red   * 255),
                        (int)(c.green * 255),
                        (int)(c.blue  * 255),
                        rightHsv
                );

                float h = rightHsv[0];

                if (h >= greenMin && h <= greenMax) rightResult = "GREEN";
                else if (h >= purpleMin && h <= purpleMax) rightResult = "PURPLE";
                else rightResult = "UNKNOWN";

                telemetry.addData("Right Hue", "%.1f", h);
            }

            // ---------------- MODE ACTIONS ----------------
            if (mode == Mode.INTAKE) {
                updateEventLogger(leftResult, true);
                updateEventLogger(rightResult, false);

                // Avoid removals due to transitions
                resetLaunchTracking();

            } else if (mode == Mode.LAUNCH) {
                updateLaunchRemoval(leftResult, leftLog, true);
                updateLaunchRemoval(rightResult, rightLog, false);

            } else {
                // IDLE
                resetEventTracking();
                resetLaunchTracking();
            }

            // ---------------- FLASH TRIGGER ----------------
            boolean x = gamepad2.x;
            if (x && !lastX) {
                // Start flash for each side only if it has something queued
                if (!leftLog.isEmpty()) {
                    leftFlashing = true;
                    leftFlashStartSec = getRuntime();
                }
                if (!rightLog.isEmpty()) {
                    rightFlashing = true;
                    rightFlashStartSec = getRuntime();
                }
            }
            lastX = x;

            // ---------------- LIGHT OUTPUTS ----------------
            // Default display = NEXT LAUNCH (newest item)
            double leftDefaultPos = posForColor(peekLaunchNext(leftLog));
            double rightDefaultPos = posForColor(peekLaunchNext(rightLog));

            double leftPos = leftDefaultPos;
            double rightPos = rightDefaultPos;

            // Override with flashing sequence when active
            if (leftFlashing) {
                double p = computeFlashPos(leftLog, leftFlashStartSec);
                if (p < -0.5) { // sentinel: done
                    leftFlashing = false;
                    leftPos = leftDefaultPos;
                } else {
                    leftPos = p;
                }
            }

            if (rightFlashing) {
                double p = computeFlashPos(rightLog, rightFlashStartSec);
                if (p < -0.5) { // sentinel: done
                    rightFlashing = false;
                    rightPos = rightDefaultPos;
                } else {
                    rightPos = p;
                }
            }

            LeftLight.setPosition(leftPos);
            RightLight.setPosition(rightPos);

            // ---------------- TELEMETRY ----------------
            telemetry.addData("Mode", mode);
            telemetry.addData("Cap cm", "%.2f", capCm);

            telemetry.addData("Left Distance cm", "%.2f", leftCm);
            telemetry.addData("Left Result", leftResult);
            telemetry.addData("Left Log (intake order)", dequeToString(leftLog));
            telemetry.addData("Left Next Launch", peekLaunchNext(leftLog));
            telemetry.addData("Left Flashing", leftFlashing);

            telemetry.addData("Right Distance cm", "%.2f", rightCm);
            telemetry.addData("Right Result", rightResult);
            telemetry.addData("Right Log (intake order)", dequeToString(rightLog));
            telemetry.addData("Right Next Launch", peekLaunchNext(rightLog));
            telemetry.addData("Right Flashing", rightFlashing);

            telemetry.update();
        }
    }

    /**
     * Logs GREEN/PURPLE as "events" in INTAKE mode.
     * - Requires STABLE_LOOPS_REQUIRED to accept a color
     * - Allows same color multiple times if there was a BREAK (UNKNOWN/TOO FAR) for BREAK_LOOPS_REQUIRED loops
     * - Logs at most once per event (so it won't spam while the object is still in front)
     */
    private void updateEventLogger(String result, boolean isLeft) {

        boolean isValid = result.equals("GREEN") || result.equals("PURPLE");

        if (isLeft) {
            if (!isValid) {
                leftStableCount = 0;
                leftLastSeen = "NONE";
                leftBreakCount++;
                if (leftBreakCount >= BREAK_LOOPS_REQUIRED) {
                    leftLoggedThisEvent = false; // ready for next event
                }
                return;
            }

            // valid color
            leftBreakCount = 0;

            if (result.equals(leftLastSeen)) leftStableCount++;
            else {
                leftLastSeen = result;
                leftStableCount = 1;
            }

            // Log once per event, after stable
            if (!leftLoggedThisEvent && leftStableCount >= STABLE_LOOPS_REQUIRED) {
                pushLog(leftLog, result);
                leftLoggedThisEvent = true;
            }

        } else {
            if (!isValid) {
                rightStableCount = 0;
                rightLastSeen = "NONE";
                rightBreakCount++;
                if (rightBreakCount >= BREAK_LOOPS_REQUIRED) {
                    rightLoggedThisEvent = false; // ready for next event
                }
                return;
            }

            // valid color
            rightBreakCount = 0;

            if (result.equals(rightLastSeen)) rightStableCount++;
            else {
                rightLastSeen = result;
                rightStableCount = 1;
            }

            // Log once per event, after stable
            if (!rightLoggedThisEvent && rightStableCount >= STABLE_LOOPS_REQUIRED) {
                pushLog(rightLog, result);
                rightLoggedThisEvent = true;
            }
        }
    }

    /**
     * In LAUNCH mode:
     * - NEXT launch color is newest->oldest = deque.peekLast()
     * - When sensor sees that next color, we "arm" removal
     * - When sensor stops seeing it (gap for LAUNCH_GAP_LOOPS_REQUIRED loops),
     *   we removeLast() from the deque.
     */
    private void updateLaunchRemoval(String sensorResult, ArrayDeque<String> log, boolean isLeft) {

        String next = peekLaunchNext(log); // "GREEN"/"PURPLE"/"NONE"
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
            } else {
                if (leftWasSeeingLaunchColor) {
                    leftLaunchGapCount++;
                    if (leftLaunchGapCount >= LAUNCH_GAP_LOOPS_REQUIRED) {
                        log.removeLast(); // launched
                        leftWasSeeingLaunchColor = false;
                        leftLaunchGapCount = 0;
                    }
                } else {
                    leftLaunchGapCount = 0;
                }
            }
        } else {
            if (seeingNext) {
                rightWasSeeingLaunchColor = true;
                rightLaunchGapCount = 0;
            } else {
                if (rightWasSeeingLaunchColor) {
                    rightLaunchGapCount++;
                    if (rightLaunchGapCount >= LAUNCH_GAP_LOOPS_REQUIRED) {
                        log.removeLast(); // launched
                        rightWasSeeingLaunchColor = false;
                        rightLaunchGapCount = 0;
                    }
                } else {
                    rightLaunchGapCount = 0;
                }
            }
        }
    }

    /**
     * Flash sequence (variable durations):
     *  - Yellow (pre) 0.25s
     *  - Color (0.50s), Yellow (0.25s), Color (0.50s), Yellow (0.25s), ...
     *  - Yellow (post) 0.25s
     *
     * Color order is LAUNCH ORDER (newest->oldest) = deque last->first.
     *
     * Returns:
     *  - servo position to set now
     *  - -1 sentinel when done
     */
    private double computeFlashPos(ArrayDeque<String> log, double startSec) {

        int n = log.size();
        if (n <= 0) return -1;

        double t = getRuntime() - startSec;

        // Pre-yellow
        if (t < YELLOW_SEC) return YELLOW_POS;
        t -= YELLOW_SEC;

        // For each color: color (0.50s) then yellow (0.25s)
        // We intentionally keep the yellow separator after each color, then also do a final post-yellow.
        for (int i = 0; i < n; i++) {

            // color i (launch order: newest->oldest)
            if (t < COLOR_SEC) {
                String colorName = getLaunchOrderItem(log, i);
                return posForColor(colorName);
            }
            t -= COLOR_SEC;

            // yellow separator
            if (t < YELLOW_SEC) return YELLOW_POS;
            t -= YELLOW_SEC;
        }

        // Post-yellow
        if (t < YELLOW_SEC) return YELLOW_POS;

        return -1; // done
    }

    private String getLaunchOrderItem(ArrayDeque<String> log, int indexFromNewest) {
        // indexFromNewest: 0 = last, 1 = second last, ...
        int target = log.size() - 1 - indexFromNewest;
        int i = 0;
        for (String s : log) {
            if (i == target) return s;
            i++;
        }
        return "NONE";
    }

    private String peekLaunchNext(ArrayDeque<String> log) {
        if (log.isEmpty()) return "NONE";
        return log.peekLast();
    }

    private double posForColor(String colorName) {
        if (colorName.equals("GREEN")) return GREEN_POS;
        if (colorName.equals("PURPLE")) return PURPLE_POS;
        if (colorName.equals("NONE")) return UNKNOWN_POS;
        return UNKNOWN_POS;
    }

    private void resetEventTracking() {
        leftLastSeen = "NONE";
        leftStableCount = 0;
        leftBreakCount = BREAK_LOOPS_REQUIRED;
        leftLoggedThisEvent = false;

        rightLastSeen = "NONE";
        rightStableCount = 0;
        rightBreakCount = BREAK_LOOPS_REQUIRED;
        rightLoggedThisEvent = false;
    }

    private void resetLaunchTracking() {
        leftWasSeeingLaunchColor = false;
        leftLaunchGapCount = 0;

        rightWasSeeingLaunchColor = false;
        rightLaunchGapCount = 0;
    }

    private void pushLog(ArrayDeque<String> log, String value) {
        while (log.size() >= MAX_LOG) log.removeFirst();
        log.addLast(value);
    }

    private String dequeToString(ArrayDeque<String> d) {
        if (d.isEmpty()) return "[]";
        StringBuilder sb = new StringBuilder("[");
        boolean first = true;
        for (String s : d) {
            if (!first) sb.append(", ");
            sb.append(s);
            first = false;
        }
        sb.append("]");
        return sb.toString();
    }
}