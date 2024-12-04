package org.firstinspires.ftc.teamcode.noncompetitional.teleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "Servos", group = "Examples")
public class ControlWristAndClawServos extends LinearOpMode {

    private Servo wristYServo;    // Servo pentru mișcare pe axa Y (sus-jos)
    private Servo wristRotServo; // Servo pentru rotație
    private Servo clawServo;     // Servo pentru clește
    private Servo elbowServo;
    private Servo elbow2Servo;

    @Override
    public void runOpMode() {


        // Inițializați hardware-ul
        wristYServo = hardwareMap.get(Servo.class, "wristYServo");
        wristRotServo = hardwareMap.get(Servo.class, "wristRotServo");
        clawServo = hardwareMap.get(Servo.class, "clawServo");
        elbowServo = hardwareMap.get(Servo.class, "elbowServo");
        elbow2Servo = hardwareMap.get(Servo.class, "elbow2Servo");

        wristYServo.setPosition(1.0);
        wristRotServo.setPosition(0.2046);
        clawServo.setPosition(0.1);
        elbowServo.setPosition(0.2451);
        elbow2Servo.setPosition(0.44);

        // Așteptați pornirea
        telemetry.addData("Status", "Waiting for start");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) { // claw
            // Controlul servo-ului de mișcare pe axa Y (sus-jos)
            if (gamepad1.right_trigger > 0.1) {
                wristYServo.setPosition(1.0); // Poziție maximă
                telemetry.addData("Wrist Y Position", "1.0 (Up)");
            } else if (gamepad1.left_trigger > 0.1) {
                wristYServo.setPosition(0.55); // Poziție intermediară
                telemetry.addData("Wrist Y Position", "0.5 (Mid)");
            }

            // Controlul servo-ului de rotație
            if (gamepad1.dpad_down) { // rotServo
                wristRotServo.setPosition(0.2046); // Rotație minimă  0.2046
                telemetry.addData("Wrist Rotation Position", "0.0 (Min)");
            } else if (gamepad1.dpad_up) {
                wristRotServo.setPosition(0.5527); // Rotație maximă  0.5527
                telemetry.addData("Wrist Rotation Position", "1.0 (Max)");
            } else if (gamepad1.dpad_left) {
                wristRotServo.setPosition(0.909); // Rotație înapoi la zero  /0.909
                telemetry.addData("Wrist Rotation Position", "0.0 (Zero)");
            }


            // Controlul servo-ului de clește (claw)
            if (gamepad1.a) { // YServo
                clawServo.setPosition(0.0); // Clește închis
                telemetry.addData("Claw Position", "0.0 (Closed)");
            } else if (gamepad1.y) {
                clawServo.setPosition(0.1); // Clește deschis`
                telemetry.addData("Claw Position", "1.0 (Open)");
            }

            if(gamepad1.right_bumper){
                elbow2Servo.setPosition(0.44);
                elbowServo.setPosition(0.2451);
                telemetry.addData("Elbow Position", "0.0 (Open)");
            }
            else if(gamepad1.left_bumper){
                elbow2Servo.setPosition(1.0);
                elbowServo.setPosition(0.8051);
                telemetry.addData("Elbow Position", "1.0 (Closed)");
            }

            if(gamepad1.right_stick_button){
                wristYServo.setPosition(1.0);
                wristRotServo.setPosition(0.909);
                clawServo.setPosition(0.1);
                elbowServo.setPosition(0.2451);
                elbow2Servo.setPosition(0.44);
            }
            else if(gamepad1.left_stick_button){
                wristYServo.setPosition(1.0);
                wristRotServo.setPosition(0.2046);
                clawServo.setPosition(0.0);
                elbowServo.setPosition(0.8051);
                elbow2Servo.setPosition(1.0);
            }

            // Afișează informații pe Driver Station
            telemetry.update();
        }
    }
}
