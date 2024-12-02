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
    public static Servo outtakeClawServo;
    public static Servo outtakeWristRollServo;
    public static Servo outtakeWristPitchServo;
    public static Servo outtakeElbowRightServo;
    public static Servo outtakeElbowLeftServo;

    public RobotConfig(HardwareMap hardwareMap) {
        frontLeftMotor = hardwareMap.dcMotor.get("frontLeftMotor");
        backLeftMotor = hardwareMap.dcMotor.get("backLeftMotor");
        frontRightMotor = hardwareMap.dcMotor.get("frontRightMotor");
        backRightMotor = hardwareMap.dcMotor.get("backRightMotor");

        outtakeClawServo = hardwareMap.servo.get("outtakeClawServo");
        outtakeWristRollServo = hardwareMap.servo.get("outtakeWristRollServo");
        outtakeWristPitchServo = hardwareMap.servo.get("outtakeWristPitchServo");
        outtakeElbowRightServo = hardwareMap.servo.get("outtakeElbowRightServo");
        outtakeElbowLeftServo = hardwareMap.servo.get("outtakeElbowLeftServo");

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

    public void setOuttakeServoPositions(double clawServoPosition,
                                         double wristRollServoPosition,
                                         double wristPitchServoPosition,
                                         double elbowRightServoPosition,
                                         double elbowLeftServoPosition) {
        outtakeClawServo.setPosition(clawServoPosition);
        outtakeWristRollServo.setPosition(wristRollServoPosition);
        outtakeWristPitchServo.setPosition(wristPitchServoPosition);
        outtakeElbowRightServo.setPosition(elbowRightServoPosition);
        outtakeElbowLeftServo.setPosition(elbowLeftServoPosition);
    }
}
