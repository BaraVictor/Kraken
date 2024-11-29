package org.firstinspires.ftc.teamcode.noncompetitional.teleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "Control Wrist and Claw Servos", group = "Examples")
public class ControlWristAndClawServos extends LinearOpMode {

    private Servo wristServo; // Servo pentru încheietură
    private Servo clawServo;  // Servo pentru clește

    @Override
    public void runOpMode() {
        // Inițializați hardware-ul
        wristServo = hardwareMap.get(Servo.class, "wristServo");
        clawServo = hardwareMap.get(Servo.class, "clawServo");

        // Așteptați pornirea
        telemetry.addData("Status", "Waiting for start");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {
            // Controlul servo-ului de încheietură (wrist)
            if (gamepad1.dpad_up) {
                wristServo.setPosition(1.0); // Poziție maximă
                telemetry.addData("Wrist Position", "1.0 (Max)");
            } else if (gamepad1.dpad_down) {
                wristServo.setPosition(0.0); // Poziție minimă
                telemetry.addData("Wrist Position", "0.0 (Min)");
            } else if (gamepad1.dpad_left || gamepad1.dpad_right) {
                wristServo.setPosition(0.5); // Poziție intermediară
                telemetry.addData("Wrist Position", "0.5 (Mid)");
            }

            // Controlul servo-ului de clește (claw)
            if (gamepad1.a) {
                clawServo.setPosition(0.0); // Clește închis
                telemetry.addData("Claw Position", "0.0 (Closed)");
            } else if (gamepad1.b) {
                clawServo.setPosition(0.5); // Clește semi-deschis
                telemetry.addData("Claw Position", "0.5 (Semi-open)");
            } else if (gamepad1.y) {
                clawServo.setPosition(1.0); // Clește deschis complet
                telemetry.addData("Claw Position", "1.0 (Open)");
            }

            // Afișează informații pe Driver Station
            telemetry.update();
        }
    }
}
