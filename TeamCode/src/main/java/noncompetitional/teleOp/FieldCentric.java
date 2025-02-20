package noncompetitional.teleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import configurations.RobotConfig;

@TeleOp(name = "Field Centric", group = "Test")
public class FieldCentric extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        // Inițializează configurația robotului
        RobotConfig robotConfig = new RobotConfig(hardwareMap);

        // Inițializează IMU
        IMU imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                RevHubOrientationOnRobot.UsbFacingDirection.UP));
        imu.initialize(parameters);

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {
            double y = -gamepad1.left_stick_y; // Mișcare înainte/înapoi
            double x = gamepad1.left_stick_x; // Mișcare stânga/dreapta
            double rx = gamepad1.right_stick_x; // Rotație

            // Resetează yaw-ul dacă este apăsat butonul "Options"
            if (gamepad1.options) {
                imu.resetYaw();
            }

            // Obține yaw-ul robotului
            double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

            // Transformă mișcarea pentru a fi relativă la orientarea robotului
            double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
            double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

            // Ajustare pentru strafing (opțional - poate fi eliminată dacă cauzează probleme)
            rotX *= 1.1;

            // Calculează puterile motoarelor
            double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
            double frontLeftPower = (rotY + rotX + rx) / denominator;
            double backLeftPower = (rotY - rotX + rx) / denominator;
            double frontRightPower = (rotY - rotX - rx) / denominator;
            double backRightPower = (rotY + rotX - rx) / denominator;

            // Setează puterile motoarelor
            robotConfig.setMotorPowers(
                    frontLeftPower,
                    backLeftPower,
                    frontRightPower,
                    backRightPower
            );
            // Debugging: afișează valorile pe ecran
            telemetry.addData("Yaw (rad)", botHeading);
            telemetry.addData("RotX", rotX);
            telemetry.addData("RotY", rotY);
            telemetry.addData("Front Left Power", frontLeftPower);
            telemetry.addData("Back Left Power", backLeftPower);
            telemetry.addData("Front Right Power", frontRightPower);
            telemetry.addData("Back Right Power", backRightPower);
            telemetry.update();
        }
    }
}
