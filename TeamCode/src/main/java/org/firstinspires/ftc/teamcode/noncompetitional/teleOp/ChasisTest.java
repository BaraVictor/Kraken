package org.firstinspires.ftc.teamcode.noncompetitional.teleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.configurations.RobotConfig;

@TeleOp(name = "ChasisTest", group = "Test")
public class ChasisTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        // Inițializează configurația robotului
        RobotConfig robotConfig = new RobotConfig(hardwareMap);

        waitForStart();

        while (opModeIsActive()) {
            if (gamepad1.a) {
                robotConfig.setMotorPowers(1, 0, 0, 0); // Testează front-left
            } else if (gamepad1.b) {
                robotConfig.setMotorPowers(0, 1, 0, 0); // Testează back-left
            } else if (gamepad1.x) {
                robotConfig.setMotorPowers(0, 0, 1, 0); // Testează front-right
            } else if (gamepad1.y) {
                robotConfig.setMotorPowers(0, 0, 0, 1); // Testează back-right
            } else {
                robotConfig.setMotorPowers(0, 0, 0, 0);
            }
        }
    }
}
