package org.firstinspires.ftc.teamcode.noncompetitional.teleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "Servo Angle Control", group = "Test")
public class ServoAngleControl extends LinearOpMode {

    private Servo servo; // Definirea servo-ului
    private Servo servo2; // Definirea altui servo-ului

    @Override
    public void runOpMode() {
        // Inițializarea hardware-ului
        servo = hardwareMap.get(Servo.class, "outtakeElbowRightServo");
        double servoPosition = servo.getPosition();
        // Așteptăm să înceapă
        telemetry.addData("Status", "Waiting for start");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {
            // Citirea valorii stick-ului stâng pe axa Y
            double stickY = -gamepad1.left_stick_y; // Inversăm direcția pentru control mai intuitiv

            // Actualizarea poziției servo-ului în funcție de valoarea stick-ului
            servoPosition += stickY * 0.001; // Incrementăm/decrementăm poziția servo-ului
            servoPosition = Math.max(0.0, Math.min(1.0, servoPosition)); // Clamping între 0.0 și 1.0
            // Setăm poziția servo-ului
            servo.setPosition(servoPosition);

            // Afișăm informații pe telemetrie
            telemetry.addData("Servo Position", servoPosition);
            telemetry.update();
        }
    }
}
