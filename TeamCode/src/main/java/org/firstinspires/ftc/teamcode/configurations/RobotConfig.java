package org.firstinspires.ftc.teamcode.configurations;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class RobotConfig {
    // Motoare de deplasare
    public static DcMotor frontLeftMotor;
    public static DcMotor backLeftMotor;
    public static DcMotor frontRightMotor;
    public static DcMotor backRightMotor;

    // Servo outtake
    public Servo outtakeClawServo;
    public Servo outtakeWristRotServo;
    public Servo outtakeWristYServo;
    public Servo outtakeElbowRightServo;
    public Servo outtakeElbowLeftServo;

    // Servo intake
    public Servo intakeElbowRightServo;
    public Servo intakeElbowLeftServo;
    public Servo intakeWristRightServo;
    public Servo intakeWristLeftServo;
    public Servo intakeWristServo;
    public Servo intakeWristRotServo;
    public Servo intakeClawServo;

    public RobotConfig(HardwareMap hardwareMap) {
        frontLeftMotor = hardwareMap.dcMotor.get("frontLeftMotor");
        backLeftMotor = hardwareMap.dcMotor.get("backLeftMotor");
        frontRightMotor = hardwareMap.dcMotor.get("frontRightMotor");
        backRightMotor = hardwareMap.dcMotor.get("backRightMotor");

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

        // Setează direcțiile motoarelor
        frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        frontRightMotor.setDirection(DcMotor.Direction.FORWARD);
        backRightMotor.setDirection(DcMotor.Direction.REVERSE);
    }

    public void setMotorPowers(double frontLeftPower, double backLeftPower, double frontRightPower, double backRightPower) {
        frontLeftMotor.setPower(frontLeftPower);
        backLeftMotor.setPower(backLeftPower);
        frontRightMotor.setPower(frontRightPower);
        backRightMotor.setPower(backRightPower);
    }

    public void setOuttakeServoPositions(double outtakeClawServoPosition,
                                  double outtakeWristRotServoPosition,
                                  double outtakeWristYServoPosition,
                                  double outtakeElbowRightServoPosition,
                                  double outtakeElbowLeftServoPosition) {
        outtakeClawServo.setPosition(outtakeClawServoPosition);
        outtakeWristRotServo.setPosition(outtakeWristRotServoPosition);
        outtakeWristYServo.setPosition(outtakeWristYServoPosition);
        outtakeElbowRightServo.setPosition(outtakeElbowRightServoPosition);
        outtakeElbowLeftServo.setPosition(outtakeElbowLeftServoPosition);
    }

    public void setIntakeServoPositions(double intakeElbowRightServoPosition,
                                        double intakeElbowLeftServoPosition,
                                        double intakeWristServoPosition,
                                        double intakeWristRightServoPosition,
                                        double intakeWristLeftServoPosition,
                                        double intakeClawServoPosition,
                                        double intakeWristRotServoPosition) {
        intakeElbowRightServo.setPosition(intakeElbowRightServoPosition);
        intakeElbowLeftServo.setPosition(intakeElbowLeftServoPosition);
        intakeWristServo.setPosition(intakeWristServoPosition);
        intakeWristRightServo.setPosition(intakeWristRightServoPosition);
        intakeWristLeftServo.setPosition(intakeWristLeftServoPosition);
        intakeClawServo.setPosition(intakeClawServoPosition);
        intakeWristRotServo.setPosition(intakeWristRotServoPosition);
    }

}
