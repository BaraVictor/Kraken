package org.firstinspires.ftc.teamcode.noncompetitional.teleOp;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "Slider Intake TeleOp", group = "TeleOp")
public class SliderIntake extends OpMode {
    // Servos for the slider intake
    private Servo leftServo;
    private Servo rightServo;

    // Constants for positions
    private static final double LEFT_SERVO_MIN = 0.0;   // Minimum position for left servo
    private static final double LEFT_SERVO_MAX = 1.0;   // Maximum position for left servo
    private static final double RIGHT_SERVO_MIN = 0.0;  // Minimum position for right servo
    private static final double RIGHT_SERVO_MAX = 1.0;  // Maximum position for right servo

    @Override
    public void init() {
        // Initialize servos from hardware map
        leftServo = hardwareMap.get(Servo.class, "leftServo");
        rightServo = hardwareMap.get(Servo.class, "rightServo");

        // Set initial positions (e.g., both at rest)
        leftServo.setPosition(0.5); // Neutral midpoint
        rightServo.setPosition(0.5); // Neutral midpoint

        telemetry.addData("Status", "Initialized");
    }

    @Override
    public void loop() {
        // Extend slider intake when pressing the 'A' button
        if (gamepad1.a) {
            extend();
            telemetry.addData("Slider Intake", "Extended");
        }
        // Retract slider intake when pressing the 'B' button
        else if (gamepad1.b) {
            retract();
            telemetry.addData("Slider Intake", "Retracted");
        }
        // Stop slider intake when pressing the 'X' button
        else if (gamepad1.x) {
            stop();
            telemetry.addData("Slider Intake", "Stopped");
        }

        telemetry.update();
    }

    /**
     * Move both servos to a specific position.
     * @param leftPosition  Desired position for the left servo (0.0 to 1.0).
     * @param rightPosition Desired position for the right servo (0.0 to 1.0).
     */
    private void setPosition(double leftPosition, double rightPosition) {
        // Clamp positions to valid ranges
        leftPosition = clamp(leftPosition, LEFT_SERVO_MIN, LEFT_SERVO_MAX);
        rightPosition = clamp(rightPosition, RIGHT_SERVO_MIN, RIGHT_SERVO_MAX);

        // Set positions
        leftServo.setPosition(leftPosition);
        rightServo.setPosition(rightPosition);
    }

    /**
     * Extend the slider intake fully.
     */
    private void extend() {
        setPosition(LEFT_SERVO_MAX, RIGHT_SERVO_MAX);
    }

    /**
     * Retract the slider intake fully.
     */
    private void retract() {
        setPosition(LEFT_SERVO_MIN, RIGHT_SERVO_MIN);
    }

    /**
     * Stop the slider intake (set both servos to neutral/midpoint).
     */
    public void stop() {
        setPosition(0.5, 0.5);
    }

    /**
     * Helper function to clamp values within a range.
     * @param value Value to clamp.
     * @param min   Minimum allowed value.
     * @param max   Maximum allowed value.
     * @return Clamped value.
     */
    private double clamp(double value, double min, double max) {
        return Math.max(min, Math.min(max, value));
    }
}
