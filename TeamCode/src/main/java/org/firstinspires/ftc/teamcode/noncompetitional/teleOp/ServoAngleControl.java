package org.firstinspires.ftc.teamcode.noncompetitional.teleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "Servo Angle Control", group = "Test")
public class ServoAngleControl extends LinearOpMode {

    private Servo servo;

    @Override
    public void runOpMode() {
        servo = hardwareMap.get(Servo.class, "outtakeClawServo");
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
        double servoPosition = servo.getPosition();
        telemetry.addData("Status", "Waiting for start");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {
            double stickY = -gamepad1.left_stick_y;

            servoPosition += stickY * 0.001;
            servoPosition = Math.max(0.0, Math.min(1.0, servoPosition));
            servo.setPosition(servoPosition);

            telemetry.addData("Servo Position", servoPosition);
            telemetry.update();
        }
    }
}
