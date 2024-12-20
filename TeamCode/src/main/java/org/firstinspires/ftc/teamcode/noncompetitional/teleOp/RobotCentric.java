package org.firstinspires.ftc.teamcode.noncompetitional.teleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.configurations.RobotConfig;

@TeleOp(name = "Robot Centric", group = "Test")
public class RobotCentric extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        // Inițializează configurația robotului
        RobotConfig robotConfig = new RobotConfig(hardwareMap);

        // Așteaptă startul opmode-ului
        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {
            // Obține valorile joystick-urilor
            double y = -gamepad1.left_stick_y; // Mișcare înainte/înapoi
            double x = gamepad1.left_stick_x; // Mișcare stânga/dreapta
            double rx = gamepad1.right_stick_x; // Rotație

            // Calculează puterile motoarelor
            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            double frontLeftPower = (y + x + rx) / denominator;
            double backLeftPower = (y - x + rx) / denominator;
            double frontRightPower = (y - x - rx) / denominator;
            double backRightPower = (y + x - rx) / denominator;

            // Setează puterea motoarelor
            robotConfig.setMotorPowers(
                    frontLeftPower,
                    backLeftPower,
                    frontRightPower,
                    backRightPower
            );

            // Debugging: afișează valorile pe ecran
            telemetry.addData("Front Left Power", frontLeftPower);
            telemetry.addData("Back Left Power", backLeftPower);
            telemetry.addData("Front Right Power", frontRightPower);
            telemetry.addData("Back Right Power", backRightPower);
            telemetry.update();
        }
    }
}
