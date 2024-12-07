package org.firstinspires.ftc.teamcode.noncompetitional.teleOp;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.teamcode.pedroPathing.util.PIDFController;
import org.firstinspires.ftc.teamcode.pedroPathing.util.CustomPIDFCoefficients;

@TeleOp(name = "Slider Control with PIDF", group = "TeleOp")
public class SliderOuttakePIDF extends OpMode {

    private DcMotorEx sliderMotor1;
    private DcMotorEx sliderMotor2;

    private PIDFController pidfController;

    private double targetPosition = 0; // Target position in ticks

    @Override
    public void init() {
        // Initialize motors
        sliderMotor1 = hardwareMap.get(DcMotorEx.class, "sliderMotor1");
        sliderMotor2 = hardwareMap.get(DcMotorEx.class, "sliderMotor2");

        // Reset encoders
        sliderMotor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        sliderMotor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        sliderMotor1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        sliderMotor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        sliderMotor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        sliderMotor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Initialize PIDF controller with coefficients
        CustomPIDFCoefficients coefficients = new CustomPIDFCoefficients(0.01, 0.0, 0.001, 0.0);
        pidfController = new PIDFController(coefficients);

        telemetry.addLine("PIDF Slider Initialized");
    }

    @Override
    public void loop() {
        // Adjust target position based on D-Pad input
        if (gamepad1.dpad_up) {
            targetPosition += 50; // Move slider up
        } else if (gamepad1.dpad_down) {
            targetPosition -= 50; // Move slider down
        }

        // Clip target position to stay within safe limits
        targetPosition = Range.clip(targetPosition, 0, 3000); // Adjust limits based on your slider setup

        // Update PIDF controller with current position and target
        pidfController.setTargetPosition(targetPosition);
        pidfController.updatePosition(sliderMotor1.getCurrentPosition());

        // Calculate motor power using PIDF output
        double power = pidfController.runPIDF();

        // Apply power to both motors
        sliderMotor1.setPower(power);
        sliderMotor2.setPower(power);

        // Telemetry for debugging
        telemetry.addData("Target Position", targetPosition);
        telemetry.addData("Current Position", sliderMotor1.getCurrentPosition());
        telemetry.addData("Motor Power", power);
        telemetry.addData("Error", pidfController.getError());
        telemetry.update();
    }
}
