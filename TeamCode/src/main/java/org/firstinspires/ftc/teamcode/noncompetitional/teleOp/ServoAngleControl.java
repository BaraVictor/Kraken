package org.firstinspires.ftc.teamcode.noncompetitional.teleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "Servo Angle Control", group = "Examples")
public class ServoAngleControl extends LinearOpMode {

    private Servo servo; // Definirea servo-ului


    @Override
    public void runOpMode() {
        // Inițializarea hardware-ului
        servo = hardwareMap.get(Servo.class, "servo");
        double servoPosition = servo.getPosition();
        // Așteptăm să înceapă
        telemetry.addData("Status", "Waiting for start");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {
            // Citirea valorii stick-ului stâng pe axa Y
            double stickY = -gamepad1.left_stick_y; // Inversăm direcția pentru control mai intuitiv

            // Actualizarea poziției servo-ului în funcție de valoarea stick-ului
            servoPosition += stickY * 0.01; // Incrementăm/decrementăm poziția servo-ului
            servoPosition = Math.max(0.0, Math.min(1.0, servoPosition)); // Clamping între 0.0 și 1.0

            // Setăm poziția servo-ului
            servo.setPosition(servoPosition);

            // Calculăm unghiul servo-ului (presupunem că range-ul servo-ului este 0-180°)
            double servoAngle = servoPosition * 180.0;

            // Afișăm informații pe telemetrie
            telemetry.addData("Servo Position", servoPosition);
            telemetry.addData("Servo Angle (degrees)", servoAngle);
            telemetry.update();
        }
    }
}
