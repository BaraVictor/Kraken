package org.firstinspires.ftc.teamcode.noncompetitional.teleOp;

import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.internal.system.Deadline;

import java.util.concurrent.TimeUnit;

@TeleOp(name = "VisionProcessor", group = "Sensor")
public class VisionProcessor extends OpMode {
    // Rate limit variables
    private static final int READ_PERIOD = 1;
    private static Deadline rate_limit;

    // HuskyLens
    private static HuskyLens huskyLens;
    private static HuskyLens.Block[] blocks;

    // Servo variable
    private Servo intakeWristRotServo;

    // Algorithms variables
    private static double W0, H0, w1, h1, degree;

    /**
     * Create a HuskyLens instance.
     */
    public VisionProcessor() {
        // Initialize HuskyLens and other variables
        rate_limit = new Deadline(READ_PERIOD, TimeUnit.SECONDS);
        W0 = 1.75;  // initial width for reference
        H0 = 4.6;  // initial height for reference
        w1 = 1.75;  // current width
        h1 = 4.6;  // current height
    }

    /**
     * Called when the OpMode is first initialized.
     */
    @Override
    public void init() {
        // Initialize the HuskyLens hardware here
        huskyLens = hardwareMap.get(HuskyLens.class, "huskyLens");

        // Initialize the servo
        intakeWristRotServo = hardwareMap.get(Servo.class, "intakeWristRotServo");
    }

    /**
     * Get the orientation of a sample which is in front of the HuskyLens in degrees.
     * @return The orientation in degrees.
     */
    public static double getSampleOrientation() {
        if (rate_limit.hasExpired()) {
            rate_limit.reset();
            blocks = huskyLens.blocks();
            for (HuskyLens.Block bbox : blocks) {
                w1 = bbox.width;
                h1 = bbox.height;

                // Adjust the logic for calculating angle based on the ratio of width and height
                double aspectRatio = w1 / h1;  // width / height ratio
                double referenceRatio = W0 / H0;  // reference aspect ratio

                // Calculate the angle based on the ratio difference
                if (aspectRatio > referenceRatio) {
                    degree = Math.toDegrees(Math.atan((aspectRatio - referenceRatio) / referenceRatio));
                } else if (aspectRatio < referenceRatio) {
                    degree = -Math.toDegrees(Math.atan((referenceRatio - aspectRatio) / referenceRatio));
                } else {
                    degree = 0;  // No rotation if aspect ratios match exactly
                }
            }
        }

        // Ensure the degree is positive and map the angle correctly
        return Math.abs(degree);  // This ensures the angle is positive
    }



    /**
     * Called repeatedly in a loop while the OpMode is running.
     */
    @Override
    public void loop() {
        // Get the sample orientation and display it on the driver station
        double orientation = getSampleOrientation();

        // Map the orientation to a servo position
        double servoPosition = getServoPositionFromAngle(orientation);

        // Set the servo position based on the orientation
        intakeWristRotServo.setPosition(servoPosition);

        // Display the angle and servo position on the driver station
        telemetry.addData("Orientation", orientation);
        telemetry.addData("Servo Position", servoPosition);
        telemetry.update();
    }

    /**
     * Maps the angle to a specific servo position.
     * - 0 to 40 degrees -> 0.29
     * - 40 to 60 degrees -> 0.57
     * - 60 to 90 degrees -> 0.84
     * @param angle The angle to map.
     * @return The corresponding servo position.
     */
    private double getServoPositionFromAngle(double angle) {
        if (angle >= 0 && angle <= 45) {
            return 0.29;
        } else if (angle > 45 && angle <= 90) {
            return 0.84;
        } else {
            return 0.29;  // Default case (to avoid undefined behavior)
        }
    }
}
