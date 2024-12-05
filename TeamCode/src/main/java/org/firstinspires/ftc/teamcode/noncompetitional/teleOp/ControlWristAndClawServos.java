package org.firstinspires.ftc.teamcode.noncompetitional.teleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "Servos", group = "Examples")
public class ControlWristAndClawServos extends LinearOpMode {

    private Servo clawServo; 
    private Servo wristRotServo;
    private Servo wristYServo;
    private Servo elbowRightServo;
    private Servo elbowLeftServo;
    private Servo intakeElbowRightServo;
    private Servo intakeElbowLeftServo;
    private Servo intakeWristRightServo;
    private Servo intakeWristLeftServo;
    private Servo intakeWristServo;

    @Override
    public void runOpMode() {
        clawServo = hardwareMap.get(Servo.class, "clawServo");
        wristRotServo = hardwareMap.get(Servo.class, "wristRotServo");
        wristYServo = hardwareMap.get(Servo.class, "wristYServo");
        elbowRightServo = hardwareMap.get(Servo.class, "elbowRightServo");
        elbowLeftServo = hardwareMap.get(Servo.class, "elbowLeftServo");

        intakeElbowRightServo = hardwareMap.get(Servo.class, "intakeElbowRightServo");
        intakeElbowLeftServo = hardwareMap.get(Servo.class, "intakeElbowLeftServo");
        intakeWristServo = hardwareMap.get(Servo.class, "intakeWristServo");
        intakeWristRightServo = hardwareMap.get(Servo.class, "intakeWristRightServo");
        intakeWristLeftServo = hardwareMap.get(Servo.class, "intakeWristLeftServo");


        clawServo.setPosition(0.94);
        wristRotServo.setPosition(0.909);
        wristYServo.setPosition(0.0435);
        elbowRightServo.setPosition(0.2451);
        elbowLeftServo.setPosition(0.44);
        
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) { // claw
            if (gamepad1.right_trigger > 0.1) {
                clawServo.setPosition(0.94);
            } else if (gamepad1.left_trigger > 0.1) {
                clawServo.setPosition(0.55); 
            }

            if (gamepad1.dpad_down) { // rotServo
                wristRotServo.setPosition(0.2046); // 0 grade
            } else if (gamepad1.dpad_up) {
                wristRotServo.setPosition(0.5527); // 90 grade
            } else if (gamepad1.dpad_left) {
                wristRotServo.setPosition(0.909); // 180 grade
            }

            if (gamepad1.a) { // YServo
                wristYServo.setPosition(0.0435); // pt transfer
            } else if (gamepad1.y) {
                wristYServo.setPosition(0.1207); // pt punere specimene
            }

            if(gamepad1.right_bumper){
                elbowRightServo.setPosition(0.2451); // ia specimen
                elbowLeftServo.setPosition(0.44);
            }
            else if(gamepad1.left_bumper){
                elbowRightServo.setPosition(0.8051); //pune specimen
                elbowLeftServo.setPosition(1.0);
            }

            if(gamepad1.right_stick_button){ //luat specimene
                clawServo.setPosition(0.94);
                wristRotServo.setPosition(0.909);
                wristYServo.setPosition(0.0435);
                elbowRightServo.setPosition(0.2451);
                elbowLeftServo.setPosition(0.44);
            }
            else if(gamepad1.left_stick_button){ //punere specimene
                clawServo.setPosition(0.94);
                wristRotServo.setPosition(0.2046);
                wristYServo.setPosition(0.1207);
                elbowRightServo.setPosition(0.8051);
                elbowLeftServo.setPosition(1.0);
            }

            if(gamepad2.dpad_down){
                intakeElbowRightServo.setPosition(0.0);
                intakeElbowLeftServo.setPosition(0.5454);

            }
            else if(gamepad2.dpad_up){
                intakeElbowRightServo.setPosition(0.5454);
                intakeElbowLeftServo.setPosition(0.0);
            }

            if(gamepad2.a){
                intakeWristRightServo.setPosition(0.0);
                intakeWristLeftServo.setPosition(1.0);
            }
            else if(gamepad2.y){
                intakeWristRightServo.setPosition(1.0);
                intakeWristLeftServo.setPosition(0.0);
            }

            if(gamepad2.right_bumper){
                intakeWristServo.setPosition(0.0);
            }
            else if(gamepad2.left_bumper){
                intakeWristServo.setPosition(1.0);
            }

            telemetry.update();
        }
    }
}
