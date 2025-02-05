package org.firstinspires.ftc.teamcode.noncompetitional.teleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.configurations.RobotConfig;

@TeleOp(name = "ChasisTest", group = "Test")
public class ChasisTest extends LinearOpMode {
    public static DcMotorEx frontLeftMotor;
    public static DcMotorEx backLeftMotor;
    public static DcMotorEx frontRightMotor;
    public static DcMotorEx backRightMotor;
    @Override
    public void runOpMode() throws InterruptedException {

        frontLeftMotor = (DcMotorEx) hardwareMap.get(DcMotor.class, "frontLeftMotor");
        backLeftMotor = (DcMotorEx) hardwareMap.get(DcMotor.class, "backLeftMotor");
        frontRightMotor = (DcMotorEx) hardwareMap.get(DcMotor.class, "frontRightMotor");
        backRightMotor = (DcMotorEx) hardwareMap.get(DcMotor.class, "backRightMotor");

        frontLeftMotor.setDirection(DcMotorEx.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotorEx.Direction.REVERSE);
        frontRightMotor.setDirection(DcMotorEx.Direction.FORWARD);
        backRightMotor.setDirection(DcMotorEx.Direction.FORWARD);

        waitForStart();

        while (opModeIsActive()) {
            if (gamepad1.a) {
                frontLeftMotor.setPower(1);
            } else if (gamepad1.b) {
                backLeftMotor.setPower(1);
            } else if (gamepad1.x) {
                frontRightMotor.setPower(1);
            } else if (gamepad1.y) {
                backRightMotor.setPower(1);
            } else {
                frontLeftMotor.setPower(0);
                backLeftMotor.setPower(0);
                frontRightMotor.setPower(0);
                backRightMotor.setPower(0);
            }
        }
    }
}
