package org.firstinspires.ftc.teamcode.noncompetitional.teleOp;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;
import com.pedropathing.util.Constants;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.opencv.core.RotatedRect;

import java.util.ArrayList;

import org.firstinspires.ftc.teamcode.configurations.RobotConfig;
import org.firstinspires.ftc.teamcode.constants.ServoConstants;
import org.firstinspires.ftc.teamcode.cvProcessors.SampleOrientationProcessor;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.FConstants;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.LConstants;

@TeleOp(name = "Camera")
@Config
public class Camera extends LinearOpMode {
    private VisionPortal visionPortal;
    private SampleOrientationProcessor processor;
    private RobotConfig robotConfig;
    private Servo intakeWristRotServo;
    private FtcDashboard dashboard;
    private boolean goToPos = true;
    private ElapsedTime canDown = new ElapsedTime();
    private boolean startTransferTimer = true;
    private ElapsedTime intakeTimer = new ElapsedTime();
    private ElapsedTime balancingTimer = new ElapsedTime();
    private ElapsedTime lineTimer = new ElapsedTime();
   private boolean transfer = false;
    private boolean canTransfer = true;
    private boolean hasTransfered = false;
    boolean closed = false;

    private static final double MIN_POSITION = 0.21;  // Fully retracted
    private static final double MAX_POSITION = 0.53;  // Fully extended
    private static final double DEAD_ZONE = 0.001;  // A tighter dead zone to prevent jitter
    private static final double STOP_THRESHOLD = 0.01;  // Threshold for stopping, when servo is close enough to center (larger range)
    private static final double MOVE_STEP_SIZE = 0.003;
    private static final double EXTEND_STEP_SIZE = 0.001;
    private static final double EXTEND_STEP_SIZE2 = 0.004;// Smaller step size to ensure smoother movement (smaller change)
    private static final long DELAY_AFTER_MOVE_MS = 30;
    double currentPosition = MIN_POSITION;
    boolean da = true;
    boolean sampleFound = false;
    boolean resetTimer = false;
    boolean retractIntake = true;
    boolean canPickup = false;
    boolean canPickup2 = false;
    double servoPos;
    private static final double X_MIN = -2.5;
    private static final double X_MAX = 0.5;
    double sampleY;
    double sampleX;
    boolean goLeft = true;
    double sampleAngle;
    boolean lineTimerReset = false;
    boolean aIntrat = false;


    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer; //?
    private int pathState;

    private PathChain line;

    private Pose startPose = new Pose(0,0,0);
    private Pose second = new Pose(0,1,0);



    @Override
    public void runOpMode() throws InterruptedException {
        robotConfig = new RobotConfig(hardwareMap);
        robotConfig.intakeWristRightServo.setPosition(ServoConstants.INTAKE_WRIST_RIGHT_CAMERA_POSITION);
        robotConfig.intakeWristLeftServo.setPosition(ServoConstants.INTAKE_WRIST_LEFT_CAMERA_POSITION);
        robotConfig.intakeElbowLeftServo.setPosition(MIN_POSITION);
        robotConfig.intakeElbowRightServo.setPosition(MIN_POSITION);
        robotConfig.outtakeClawServo.setPosition(ServoConstants.OUTTAKE_CLAW_OPEN_POSITION);
        robotConfig.intakeClawServo.setPosition(ServoConstants.INTAKE_CLAW_OPEN_POSITION);
        robotConfig.outtakeElbowLeftServo.setPosition(ServoConstants.OUTTAKE_ELBOW_LEFT_PICKUP_POSITION);
        robotConfig.outtakeElbowRightServo.setPosition(ServoConstants.OUTTAKE_ELBOW_RIGHT_PICKUP_POSITION);
        robotConfig.outtakeWristYServo.setPosition(ServoConstants.OUTTAKE_WRIST_Y_TRANSFER_POSITION);
        robotConfig.intakeWristServo.setPosition(ServoConstants.INTAKE_WRIST_PERPENDICULAR);
        dashboard = FtcDashboard.getInstance();
        processor = new SampleOrientationProcessor(telemetry);
        intakeWristRotServo = hardwareMap.get(Servo.class, "intakeWristRotServo");
        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "camera")) // Your webcam name
                .addProcessor(processor)
                .build();// Pass telemetry if needed

        // Example data: You need actual RotatedRects from image processing
        ArrayList<RotatedRect> detectedRects = new ArrayList<>();

        // Example scaling factor (change based on your image size)
        double scalingFactor = 1;

        pathTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();

        Constants.setConstants(FConstants.class, LConstants.class);
        follower = new Follower(hardwareMap);
        follower.setStartingPose(startPose);

        line = follower.pathBuilder()
                .addPath(new BezierLine(new com.pedropathing.pathgen.Point(startPose), new Point(second)))
                .setConstantHeadingInterpolation(startPose.getHeading())
                        .build();


        // Call getOffsets()
        waitForStart();

        while (opModeIsActive()) {
            follower.update();
            telemetry.addData("lineTimer", lineTimer.seconds());
            telemetry.addData("aIntrat", aIntrat);
            telemetry.addData("balancingTimer", balancingTimer.seconds());
            telemetry.addData("da", da);
            ArrayList<double[]> samples = processor.getRealPositions();
           ArrayList<Double> sampleAngles = processor.getSampleAngles();
            follower.update();
            for(int i = 0; i < samples.size(); i++) {
                if (i < sampleAngles.size()) {
                    sampleX = samples.get(i)[0];  // X position of the sample
                    sampleY = samples.get(i)[1];
                    if(sampleX <= X_MIN && goLeft && sampleY <=3.5){
//                        sampleAngle = processor.getSampleAngles().get(i);
                        goLeft = false;
                        sampleFound = true;
                        follower.followPath(line);
                    }
                    // If a sample is found within the range of X, stop extending and center the servo
//                    if (sampleX >= X_MIN && sampleX <= X_MAX && sampleY <= 3.5) {
//                        double sampleAngle = processor.getSampleAngles().get(i);
//                        servoPos = mapAngleToServoPosition(sampleAngle);
//                        sampleFound = true;
//                        telemetry.addData("sampleFound", sampleFound);
//                    }
                }
            }
            if(da && !goLeft){
                da= false;
                lineTimer.reset();
                lineTimerReset = true;
            }
            if(lineTimer.seconds()>3 && lineTimerReset){
                canPickup = true;
            }
            if(sampleFound && resetTimer && canPickup) {
                balancingTimer.reset();
                resetTimer = false;
            }

            if(!sampleFound){
                    // Gradually extend the arm until a sample is found
                extendArmGradually();
            }
            if(sampleFound && balancingTimer.seconds()>1 && canPickup && !resetTimer){
                currentPosition = robotConfig.intakeElbowLeftServo.getPosition();
                if(retractIntake && balancingTimer.seconds()>2) {
                    aIntrat = true;
                    servoPos = mapAngleToServoPosition(processor.getFirstSampleAngle());
                    intakeWristRotServo.setPosition(servoPos);
                    robotConfig.intakeElbowLeftServo.setPosition(currentPosition - 0.04);
                    robotConfig.intakeElbowRightServo.setPosition(currentPosition - 0.04);
                    retractIntake = false;
                    canTransfer = false;
                    servoPos = mapAngleToServoPosition(sampleAngle);
                    robotConfig.intakeWristLeftServo.setPosition(ServoConstants.INTAKE_WRIST_LEFT_HOVER_POSITION);
                    robotConfig.intakeWristRightServo.setPosition(ServoConstants.INTAKE_WRIST_RIGHT_HOVER_POSITION);
                }
                if(!canTransfer) {
                    robotConfig.intakeWristServo.setPosition(ServoConstants.INTAKE_WRIST_DOWN);
                    if (balancingTimer.seconds() > 3) {
                        robotConfig.intakeWristLeftServo.setPosition(ServoConstants.INTAKE_WRIST_LEFT_DOWN_POSITION);
                        robotConfig.intakeWristRightServo.setPosition(ServoConstants.INTAKE_WRIST_RIGHT_DOWN_POSITION);
                    }
                    if (balancingTimer.seconds() > 4) {
                        robotConfig.intakeClawServo.setPosition(ServoConstants.INTAKE_CLAW_CLOSED_POSITION);
                        balancingTimer.reset();
                        canTransfer = true;
                    }
                }
                if(canTransfer && balancingTimer.seconds()>2)
                    transfer(0.6);
            }
            telemetry.update();// Delay per iteration to ensure smooth motion
        }
        visionPortal.close();
    }

    public double mapAngleToServoPosition(double angle) {
        double newAngle = angle * 180 / Math.PI;
        double servoMin = 0.39, servoMax = 0.98;
        //newAngle = Math.max(0, Math.min(180, newAngle));
        return 0.00328 * newAngle + 0.39;
    }
    private void extendArmGradually() {
        // Gradually extend the arm until a sample is detected
        if (currentPosition < MAX_POSITION) {
            currentPosition += EXTEND_STEP_SIZE2;
            currentPosition = Math.min(currentPosition, MAX_POSITION);
            robotConfig.intakeElbowLeftServo.setPosition(currentPosition);
            robotConfig.intakeElbowRightServo.setPosition(currentPosition);
            sleep(20); // Delay to smooth the extension
        }
    }
    private void smoothMoveServoToPosition(double targetServoPosition) {
        // Smoothly move the servo to the target position (based on Y of the sample)
        double currentPosition = robotConfig.intakeElbowLeftServo.getPosition();

        if (Math.abs(currentPosition - targetServoPosition) > DEAD_ZONE) {
            if (Math.abs(currentPosition - targetServoPosition) > STOP_THRESHOLD) {
                if (currentPosition < targetServoPosition) {
                    // Move the servo towards the target if it's behind
                    robotConfig.intakeElbowLeftServo.setPosition(Math.min(currentPosition + MOVE_STEP_SIZE, targetServoPosition));
                    robotConfig.intakeElbowRightServo.setPosition(Math.min(currentPosition + MOVE_STEP_SIZE, targetServoPosition));
                } else {
                    // Move the servo towards the target if it's ahead
                    robotConfig.intakeElbowLeftServo.setPosition(Math.max(currentPosition - MOVE_STEP_SIZE, targetServoPosition));
                    robotConfig.intakeElbowRightServo.setPosition(Math.max(currentPosition - MOVE_STEP_SIZE, targetServoPosition));
                }
                sleep(20); // Small delay to smooth the movement
            }
        } else {
            // If within the dead zone, stop adjusting the servo
            robotConfig.intakeElbowLeftServo.setPosition(targetServoPosition);
            robotConfig.intakeElbowRightServo.setPosition(targetServoPosition);
        }
    }
    private double mapYToServoPosition(double sampleY) {
        // You can adjust the mapping logic here depending on how your servo should behave
        // For simplicity, we assume a direct mapping for the Y value (just scale it appropriately)
        double Y_MIN = -5.0; // Adjust this to the minimum Y position your system can handle
        double Y_MAX = 5.0;  // Adjust this to the maximum Y position your system can handle

        // Linearly map the sample Y position to the servo range
        double rangeY = Y_MAX - Y_MIN;
        double rangeServo = MAX_POSITION - MIN_POSITION;

        // Calculate the scaled position of the servo based on the Y position
        double scaledPosition = ((sampleY - Y_MIN) / rangeY) * rangeServo + MIN_POSITION;

        // Ensure the calculated position is within the valid servo range
        return Math.max(MIN_POSITION, Math.min(scaledPosition, MAX_POSITION));
    }
    public double mapYOffsetToServoPosition(double yoffset) {
        double[] yoffsets = {-0.5, -0.4, -0.3, -0.2, -0.1, 0, 0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9};
        double[] servoPositions = {0.17, 0.2, 0.22, 0.24, 0.26, 0.27, 0.29, 0.29, 0.3, 0.31, 0.31, 0.32, 0.35, 0.37, 0.39};
        yoffset = Math.max(-0.5, Math.min(0.9, yoffset));

        for (int i = 0; i < yoffsets.length - 1; i++) {
            if (yoffset >= yoffsets[i] && yoffset <= yoffsets[i + 1]) {
                double x1 = yoffsets[i], x2 = yoffsets[i + 1];
                double y1 = servoPositions[i], y2 = servoPositions[i + 1];
                return y1 + ((yoffset - x1) * (y2 - y1)) / (x2 - x1);
            }
        }
        return (yoffset == -0.5) ? 0.17 : (yoffset == 0.9) ? 0.39 : 0.27;
    }

    private void transfer(double timer) {
        if (startTransferTimer) {
            intakeTimer.reset();
            closed = false;
            startTransferTimer = false;
            transfer = false;
        }
        if (!transfer) {
            robotConfig.setIntakeServoPositions(
                    ServoConstants.INTAKE_ELBOW_RIGHT_RETRACTED_POSITION,
                    ServoConstants.INTAKE_ELBOW_LEFT_RETRACTED_POSITION,
                    ServoConstants.INTAKE_WRIST_UP,
                    ServoConstants.INTAKE_WRIST_RIGHT_UP_POSITION,
                    ServoConstants.INTAKE_WRIST_LEFT_UP_POSITION,
                    ServoConstants.INTAKE_CLAW_CLOSED_POSITION,
                    ServoConstants.INTAKE_WRIST_ROT_0_DEGREES
            );
            transfer = true;
        }
        if (transfer && intakeTimer.seconds() > timer+0.1) {
            if (!closed) {
                robotConfig.setOuttakeServoPositions(
                        ServoConstants.OUTTAKE_CLAW_CLOSED_POSITION,
                        ServoConstants.OUTTAKE_WRIST_ROT_180_DEGREES,
                        ServoConstants.OUTTAKE_WRIST_Y_TRANSFER_POSITION,
                        ServoConstants.OUTTAKE_ELBOW_RIGHT_PICKUP_POSITION,
                        ServoConstants.OUTTAKE_ELBOW_LEFT_PICKUP_POSITION
                );
                closed = true;
            }
            if (intakeTimer.seconds() > timer + 0.3) {
                robotConfig.setIntakeServoPositions(
                        ServoConstants.INTAKE_ELBOW_RIGHT_RETRACTED_POSITION,
                        ServoConstants.INTAKE_ELBOW_LEFT_RETRACTED_POSITION,
                        ServoConstants.INTAKE_WRIST_UP,
                        ServoConstants.INTAKE_WRIST_RIGHT_UP_POSITION,
                        ServoConstants.INTAKE_WRIST_LEFT_UP_POSITION,
                        ServoConstants.INTAKE_CLAW_OPEN_POSITION,
                        ServoConstants.INTAKE_WRIST_ROT_0_DEGREES
                );
                hasTransfered = true;
            }
        }
    }
}
