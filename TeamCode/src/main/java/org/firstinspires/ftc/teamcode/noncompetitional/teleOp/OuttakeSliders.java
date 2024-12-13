package org.firstinspires.ftc.teamcode.noncompetitional.teleOp;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.pedroPathing.util.CustomPIDFCoefficients;
import org.firstinspires.ftc.teamcode.pedroPathing.util.PIDFController;

@Config  // Enables the adjustment of values from the FTC Dashboard
@TeleOp(name = "Outtake Sliders with PIDF and Joystick", group = "TeleOp")
public class OuttakeSliders extends OpMode {
    private DcMotorEx leftMotor;
    private DcMotorEx rightMotor;

    private FtcDashboard dashboard;  // Instance of the Dashboard

    private PIDFController pidfControllerLeft;
    private PIDFController pidfControllerRight;

    // Limits for the target position
    private static final int MIN_POSITION = 0;       // Minimum position in encoder units
    private static final int MAX_POSITION = 3990;    // Maximum position in encoder units
    private static final int TOLERANCE = 10;         // Tolerance to stop the motor (in encoder ticks)


    // PIDF Coefficients adjustable via the FTC Dashboard
    public static double P = 0.01;
    public static double I = 0.0001;
    public static double D = 0.0003;
    public static double F = 0.008;
    public static double K =0;

    public static double targetPosition = 0;  // This will be adjustable via Dashboard

    @Override
    public void init() {
        // Initialize motors
        leftMotor = hardwareMap.get(DcMotorEx.class, "upMotor");
        rightMotor = hardwareMap.get(DcMotorEx.class, "downMotor");

        // Reset encoders
        leftMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        rightMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

        leftMotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        rightMotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        leftMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        rightMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        // Initialize PIDF controllers
        updatePIDFController();  // Update PIDF with current coefficients

        // Initialize FTC Dashboard
        dashboard = FtcDashboard.getInstance();
        dashboard.setTelemetryTransmissionInterval(25); // Set telemetry update frequency
        telemetry = dashboard.getTelemetry();  // Connect telemetry to the dashboard

        telemetry.addLine("Outtake Sliders Initialized with PIDF and Joystick Control");
    }

    @Override
    public void loop() {
        // Clip target position to stay within safe limits
        targetPosition = Range.clip(targetPosition, MIN_POSITION, MAX_POSITION);

        // Update PIDF controllers with current position and target
        pidfControllerLeft.setTargetPosition(targetPosition);
        pidfControllerLeft.updatePosition(leftMotor.getCurrentPosition());

        pidfControllerRight.setTargetPosition(targetPosition);
        pidfControllerRight.updatePosition(rightMotor.getCurrentPosition());

        // Calculate motor power using PIDF output
        double powerLeft = pidfControllerLeft.runPIDF() + K;
        double powerRight = pidfControllerRight.runPIDF() + K;

        // Stop motors if they are within tolerance of the target position
        if (Math.abs(leftMotor.getCurrentPosition() - targetPosition) <= TOLERANCE) {
            powerLeft = 0;
        }
        if (Math.abs(rightMotor.getCurrentPosition() - targetPosition) <= TOLERANCE) {
            powerRight = 0;
        }

        // Apply power to both motors
        leftMotor.setPower(powerLeft);
        rightMotor.setPower(powerRight);

        if(-gamepad1.left_stick_y > 0.1) {
            targetPosition +=10;
        }
        else if(-gamepad1.left_stick_y < -0.1) {
            targetPosition -=10;
        }

        // Transmit telemetry data to FTC Dashboard
        telemetry.addData("Target Position", targetPosition);
        telemetry.addData("Left Motor Position", leftMotor.getCurrentPosition());
        telemetry.addData("Right Motor Position", rightMotor.getCurrentPosition());
        telemetry.addData("Left Motor Power", powerLeft);
        telemetry.addData("Right Motor Power", powerRight);
        telemetry.addData("Left Motor Error", leftMotor.getCurrentPosition() - targetPosition);
        telemetry.addData("Right Motor Error", rightMotor.getCurrentPosition() - targetPosition);
        telemetry.update();
    }

    // Method to update PIDF coefficients from Dashboard
    private void updatePIDFController() {
        CustomPIDFCoefficients coefficients = new CustomPIDFCoefficients(P, I, D, F);
        pidfControllerLeft = new PIDFController(coefficients);
        pidfControllerRight = new PIDFController(coefficients);
    }
}
