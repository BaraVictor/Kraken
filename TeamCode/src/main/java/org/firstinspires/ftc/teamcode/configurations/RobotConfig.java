package org.firstinspires.ftc.teamcode.configurations;
import android.graphics.Color;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class RobotConfig {
    // Motoare de deplasare
    public static DcMotorEx frontLeftMotor;
    public static DcMotorEx backLeftMotor;
    public static DcMotorEx frontRightMotor;
    public static DcMotorEx backRightMotor;

    // Motoare de outtake
    public static DcMotorEx upMotor;
    public static DcMotorEx downMotor;

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

    float[] hsvValues = {0F, 0F, 0F};
    private ColorSensor colorSensor;    // REV Color Sensor V3
    private DistanceSensor distanceSensor; // Proximity sensor

    public RobotConfig(HardwareMap hardwareMap) {
        frontLeftMotor = (DcMotorEx) hardwareMap.get(DcMotor.class, "frontLeftMotor");
        backLeftMotor = (DcMotorEx) hardwareMap.get(DcMotor.class, "backLeftMotor");
        frontRightMotor = (DcMotorEx) hardwareMap.get(DcMotor.class, "frontRightMotor");
        backRightMotor = (DcMotorEx) hardwareMap.get(DcMotor.class, "backRightMotor");

        upMotor = (DcMotorEx) hardwareMap.get(DcMotor.class, "upMotor");
        downMotor = (DcMotorEx) hardwareMap.get(DcMotor.class, "downMotor");

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

        colorSensor = hardwareMap.get(ColorSensor.class, "colorSensor");
        distanceSensor = hardwareMap.get(DistanceSensor.class, "colorSensor");

        // Setează direcțiile motoarelor
        frontLeftMotor.setDirection(DcMotorEx.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotorEx.Direction.REVERSE);
        frontRightMotor.setDirection(DcMotorEx.Direction.FORWARD);
        backRightMotor.setDirection(DcMotorEx.Direction.FORWARD);

        upMotor.setDirection(DcMotorEx.Direction.REVERSE);
        downMotor.setDirection(DcMotorEx.Direction.REVERSE);

        upMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        downMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

        upMotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        downMotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        upMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        downMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
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

    public void colors (){
        Color.RGBToHSV(
                (int) (colorSensor.red() * 255.0 / 1023.0),
                (int) (colorSensor.green() * 255.0 / 1023.0),
                (int) (colorSensor.blue() * 255.0 / 1023.0),
                hsvValues
        );
    }

    public double getDistance(){
        return distanceSensor.getDistance(DistanceUnit.CM);
    }

    public double getCohue(){
        return hsvValues[0];
    }
    public double getSaturation(){
        return hsvValues[1];
    }
    public double getValue(){
        return hsvValues[2];
    }

}
