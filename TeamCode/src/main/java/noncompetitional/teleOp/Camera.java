package noncompetitional.teleOp;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.opencv.core.Point;
import org.opencv.core.RotatedRect;

import java.util.ArrayList;

import cvProcessors.SampleOrientationProcessor;

@TeleOp(name = "Camera")
@Config
public class Camera extends LinearOpMode {
    private VisionPortal visionPortal;
    private SampleOrientationProcessor processor;

    private Servo intakeWristRotServo;

    private FtcDashboard dashboard;
    @Override
    public void runOpMode() throws InterruptedException {
        dashboard = FtcDashboard.getInstance();
        processor = new SampleOrientationProcessor(telemetry);
        intakeWristRotServo = hardwareMap.get(Servo.class, "intakeWristRotServo");
        visionPortal = VisionPortal.easyCreateWithDefaults(
                hardwareMap.get(WebcamName.class, "camera"), // Change if using another camera
                processor
        );
        waitForStart();
        while (opModeIsActive()) {
            ArrayList<Point> detectedObjects = processor.getOffsets();
            // Get the detected sample angle
            double sampleAngle = processor.getSampleAngle();
            double sampleAngleDegrees = sampleAngle * 180 / Math.PI;
            if(gamepad1.a) {
                double servoPosition = mapAngleToServoPosition(sampleAngleDegrees);
                intakeWristRotServo.setPosition(servoPosition);
            }

            if (!detectedObjects.isEmpty()) {
                Point largestObject = detectedObjects.get(0);

                for (Point obj : detectedObjects) {
                    if (obj.y > largestObject.y) { // Assuming lower Y = larger object
                        largestObject = obj;
                    }
                }

                telemetry.addData("Tracking Object", largestObject);
            }

            // Get the detected object positions (scaled in inches)
            ArrayList<Point> offsets = processor.getOffsets();

            // Display telemetry data
            telemetry.addData("Sample Angle (radians)", sampleAngle);
            telemetry.addData("Sample Angle (degrees)", sampleAngleDegrees);
            telemetry.addData("Offsets", offsets.toString());
            telemetry.update();

        }

        // Stop the vision processing when OpMode ends
        visionPortal.close();

    }
    public double mapAngleToServoPosition(double angle) {
        // Define the min and max values for angle and servo position
        double angleMin = 0;
        double angleMax = 90;
        double servoMin = 0.29;
        double servoMax = 0.86;

        // Ensure the angle is within bounds (between 0 and 90)
        if(angle<0)
            angle = 180-angle;

        // Ensure the angle is within bounds (between 0 and 90)
        angle = Math.max(angleMin, Math.min(angleMax, angle));

        // Apply the linear mapping formula with the reversed servo position range
        double servoPosition = servoMax - ((angle - angleMin) / (angleMax - angleMin)) * (servoMax - servoMin);


        return servoPosition;
    }
}