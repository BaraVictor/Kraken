package org.firstinspires.ftc.teamcode.noncompetitional.teleOp;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.configurations.RobotConfig;
import org.firstinspires.ftc.teamcode.constants.OuttakeConstants;
import org.firstinspires.ftc.teamcode.pedroPathing.util.CustomPIDFCoefficients;
import org.firstinspires.ftc.teamcode.pedroPathing.util.PIDFController;

@Config  // Enables the adjustment of values from the FTC Dashboard
@TeleOp(name = "Outtake Sliders with PIDF and Joystick", group = "Test")
public class OuttakeSliders extends OpMode {

    private FtcDashboard dashboard;

    private RobotConfig robotConfig;// Instance of the Dashboard

    private PIDFController pidfControllerUp;

    private boolean areSlidesDown = true;

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
        robotConfig = new RobotConfig(hardwareMap);

        MotorConfigurationType motorConfigurationTypeUpMotor = RobotConfig.upMotor.getMotorType().clone();
        motorConfigurationTypeUpMotor.setAchieveableMaxRPMFraction(1.0);
        RobotConfig.upMotor.setMotorType(motorConfigurationTypeUpMotor);

        MotorConfigurationType motorConfigurationTypeMidMotor = RobotConfig.midMotor.getMotorType().clone();
        motorConfigurationTypeMidMotor.setAchieveableMaxRPMFraction(1.0);
        RobotConfig.midMotor.setMotorType(motorConfigurationTypeMidMotor);

        MotorConfigurationType motorConfigurationTypeDownMotor = RobotConfig.downMotor.getMotorType().clone();
        motorConfigurationTypeDownMotor.setAchieveableMaxRPMFraction(1.0);
        RobotConfig.downMotor.setMotorType(motorConfigurationTypeDownMotor);
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
        pidfControllerUp.setTargetPosition(targetPosition);
        pidfControllerUp.updatePosition(robotConfig.upMotor.getCurrentPosition());
        // Calculate motor power using PIDF output
        double powerLeft = pidfControllerUp.runPIDF() + K;

        // Stop motors if they are within tolerance of the target position
        if (Math.abs(robotConfig.upMotor.getCurrentPosition() - targetPosition) <= TOLERANCE) {
            powerLeft = 0;
        }
        if(robotConfig.upMotor.getCurrentPosition()>10)
            areSlidesDown = false;
        if(robotConfig.upMotor.getVelocity()<0.1 && robotConfig.upMotor.getCurrentPosition()<10){
            RobotConfig.upMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
            RobotConfig.midMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
            RobotConfig.downMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

            RobotConfig.upMotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
            RobotConfig.midMotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
            RobotConfig.downMotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
            areSlidesDown = true;
        }
        if(targetPosition == OuttakeConstants.OUTTAKE_MIN_POSITION && areSlidesDown){
            robotConfig.upMotor.setPower(0);
            robotConfig.midMotor.setPower(0);
            robotConfig.downMotor.setPower(0);
        }
        if(targetPosition == OuttakeConstants.OUTTAKE_MIN_POSITION && robotConfig.upMotor.getCurrentPosition()>10){
            robotConfig.upMotor.setPower(-0.9);
            robotConfig.midMotor.setPower(-0.9);
            robotConfig.downMotor.setPower(-0.9);

        }
        else {
            robotConfig.upMotor.setPower(powerLeft);
            robotConfig.midMotor.setPower(powerLeft);
            robotConfig.downMotor.setPower(powerLeft);
        }
        robotConfig.upMotor.setPower(powerLeft);
        robotConfig.midMotor.setPower(powerLeft);
        robotConfig.downMotor.setPower(powerLeft);
        // Apply power to both motors


        if(-gamepad1.left_stick_y > 0.1) {
            targetPosition +=10;
        }
        else if(-gamepad1.left_stick_y < -0.1) {
            targetPosition -=10;
        }

        // Transmit telemetry data to FTC Dashboard
        telemetry.addData("velocity", RobotConfig.upMotor.getVelocity());
        telemetry.addData("areSlidesDown", areSlidesDown);
        telemetry.addData("Target Position", targetPosition);
        telemetry.addData("Left Motor Position", robotConfig.upMotor.getCurrentPosition());


        telemetry.addData("Left Motor Error", robotConfig.upMotor.getCurrentPosition() - targetPosition);
        telemetry.addData("Right Motor Error", robotConfig.downMotor.getCurrentPosition() - targetPosition);

        telemetry.addData("Right Motor Power", robotConfig.downMotor.getPower());
        telemetry.addData("Left Motor Power", robotConfig.upMotor.getPower());
        telemetry.update();
    }

    // Method to update PIDF coefficients from Dashboard
    private void updatePIDFController() {
        CustomPIDFCoefficients coefficients = new CustomPIDFCoefficients(P, I, D, F);
        pidfControllerUp = new PIDFController(coefficients);
    }
}
