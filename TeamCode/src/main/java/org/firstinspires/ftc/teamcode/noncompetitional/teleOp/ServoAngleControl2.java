package org.firstinspires.ftc.teamcode.noncompetitional.teleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "Servo Angle Control 2", group = "Test")
public class ServoAngleControl2 extends LinearOpMode {

    private Servo servo;
    private Servo servo2;

    @Override
    public void runOpMode() {
        servo = hardwareMap.get(Servo.class, "intakeElbowRightServo");
        servo2 = hardwareMap.get(Servo.class, "intakeElbowLeftServo");
        /*
        servo = hardwareMap.get(Servo.class, "outtakeWristRotServo");
        servo = hardwareMap.get(Servo.class, "outtakeWristYServo");
        servo = hardwareMap.get(Servo.class, "outtakeElbowRightServo");
        servo = hardwareMap.get(Servo.class, "outtakeElbowLeftServo");
        servo = hardwareMap.get(Servo.class, "intakeElbowRightServo");
        servo = hardwareMap.get(Servo.class, "intakeElbowLeftServo");
        servo = hardwareMap.get(Servo.class, "intakeWristServo");
        servo = hardwareMap.get(Servo.class, "intakeWristRightServo");
        servo = hardwareMap.get(Servo.class, "intakeWristLeftServo");
        servo = hardwareMap.get(Servo.class, "intakeClawServo");
        servo = hardwareMap.get(Servo.class, "intakeWristRotServo");
        */
        double servoPosition = 0;
        double servoPosition2 = 0;
        telemetry.addData("Status", "Waiting for start");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {
            double stickY = -gamepad1.left_stick_y;
            if(gamepad1.a){
                servoPosition = 0.096;
                servoPosition2 = 0.096;
            }
            if(gamepad1.b){
                servoPosition = 0.526;
                servoPosition2 = 0.526;
            }
            /*if(gamepad1.x){
                servoPosition = 1;
                servoPosition2 = 1;
            }*/

            servoPosition += stickY * 0.001;
            servoPosition2 += stickY * 0.001;
            servoPosition = Math.max(0.0, Math.min(1.0, servoPosition));
            servoPosition2 = Math.max(0.0, Math.min(1.0, servoPosition2));
            servo.setPosition(servoPosition);
            servo2.setPosition(servoPosition2);

            telemetry.addData("Servo Position", servoPosition);
            telemetry.addData("Servo Position 2", servoPosition2);
            telemetry.addData("RealServoPosition", servo.getPosition());
            telemetry.addData("RealServoPosition2", servo2.getPosition());
            telemetry.update();
        }
    }
}
