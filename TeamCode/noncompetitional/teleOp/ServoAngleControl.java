package org.firstinspires.ftc.teamcode.noncompetitional.teleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;


@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "Servo Angle Control", group = "Test")
public class ServoAngleControl extends LinearOpMode {

    private Servo servo;

    @Override
    public void runOpMode() {
        servo = hardwareMap.get(Servo.class, "outtakeElbowLeftServo");

        /*
        outtakeClawServo = hardwareMap.get(Servo.class, "outtakeClawServo");
        outtakeWristRotServo = hardwareMap.get(Servo.class, "outtakeWristRotServo");
        outtakeWristYServo = hardwareMap.get(Servo.class, "outtakeWristYServo");
        outtakeElbowRightServo = hardwareMap.get(Servo.class, "outtakeElbowRightServo");
        outtakeElbowLeftServo = hardwareMap.get(Servo.class, "outtakeElbowLeftServo");

        intakeElbowRightServo = hardwareMap.get(Servo.class, "intakeElbowRightServo");
        intakeElbowLeftServo = hardwareMap.get(Servo.class, "intakeElbowLeftServo");
        intakeWristServo = hardwareMap.get(Servo.class, "intakeWristServo");
        intakeWristRightServo = hardwareMap.get(Servo.class, "intakeWristRightServo");
        intakeWristLeftServo = hardwareMap.get(Servo.class, "intakeWristLeftServo");
        intakeClawServo = hardwareMap.get(Servo.class, "intakeClawServo");
        intakeWristRotServo = hardwareMap.get(Servo.class, "intakeWristRotServo");
        */
        double servoPosition = servo.getPosition();
        telemetry.addData("Status", "Waiting for start");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {
            double stickY = -gamepad1.left_stick_y;

            if(gamepad1.a){
                servoPosition = 0;
            }
            if(gamepad1.b){
                servoPosition = 0.5;
            }
            if(gamepad1.x){
                servoPosition = 1;
            }

            servoPosition += stickY * 0.001;
            servoPosition = Math.max(0.0, Math.min(1.0, servoPosition));
            servo.setPosition(servoPosition);

            telemetry.addData("Servo Position", servoPosition);
            telemetry.addData("Real Servo Position", servo.getPosition());
            telemetry.update();
        }
    }
}
