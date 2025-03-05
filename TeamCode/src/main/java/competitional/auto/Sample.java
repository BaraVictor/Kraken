package competitional.auto;

import static java.lang.Math.toRadians;

import com.acmerobotics.dashboard.FtcDashboard;
import com.pedropathing.util.Constants;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import  com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import configurations.RobotConfig;
import com.pedropathing.follower.*;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierCurve;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.Path;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;
import com.pedropathing.util.Timer;
import com.pedropathing.util.CustomPIDFCoefficients;
import com.pedropathing.util.PIDFController;

import constants.OuttakeConstants;
import constants.ServoConstants;
import cvProcessors.SampleOrientationProcessor;
import pedroPathing.constants.FConstants;
import pedroPathing.constants.LConstants;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;

import java.util.ArrayList;

//@Config
@Autonomous(name = "🐙 SampleAuto (4+0) 🐙", group = "A. Competitional")
public class Sample extends OpMode {

    private ElapsedTime retractTimer = new ElapsedTime();
    private ElapsedTime hoverTimer = new ElapsedTime();
    private ElapsedTime intakeTimer = new ElapsedTime();
    private ElapsedTime turningTimer = new ElapsedTime();
    private ElapsedTime revolutTimer = new ElapsedTime();

    private boolean deposited = false;
    private boolean hovering = false;
    private boolean intaking = false;
    private boolean transfer = false;
    private boolean hasTurned = false;
    private boolean closed = false;
    private boolean opened = false;
    private boolean startTimer = true;
    private boolean startHoverTimer = true;
    private boolean startTransferTimer = true;
    private int numarare = 0;
    private boolean da = false;

    private RobotConfig robotConfig;

    private FtcDashboard dashboard;

    private boolean areSlidesDown = false;
    private PIDFController pidfController;

    private PIDFController pidfControllerUp;
    private PIDFController pidfControllerDown;

    public static double P = 0.007; //was 0.0128
    public static double I = 0;
    public static double D = 0; //was 0.00005
    public static double F = 0;    //was                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                  ;      //was 0.01
    public static double K = 0;
    public static double targetPosition = 0;

    public static boolean hasScored = false;
    public static boolean hasTransfered = false;
    public static boolean hasFinishedPickUp = false;

    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer; //?
    private int pathState;


    private final Pose startPose = new Pose(7.5, 111, toRadians(270));

    private final Pose score = new Pose(21, 132, toRadians(315));

    private final Pose sample1 = new Pose(28.5, 129, toRadians(0));

    private final Pose sample2 = new Pose(28.5, 138.5, toRadians(0));

    private final Pose sample3 = new Pose(33.5, 138, toRadians(45));

    private final Pose sub1 = new Pose(65, 98, Math.toRadians(-90));

    private final Pose subControl = new Pose(62, 120, Math.toRadians(-90));

    private final Pose parkFinal = new Pose(75, 93, Math.toRadians(90));


    private PathChain scorePreload, firstPickup, secondPickup, thirdPickup, subPath, score1, score2, score3; //?

    public void buildPaths() {

        /* This is our grabPickup1 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        scorePreload = follower.pathBuilder()
                .addPath(new BezierLine(new Point(startPose), new Point(score)))
                .setLinearHeadingInterpolation(startPose.getHeading(), score.getHeading())
                .setZeroPowerAccelerationMultiplier(4.0)
                .addParametricCallback(0, () -> follower.setMaxPower(0.9))
                .setPathEndTimeoutConstraint(0)
                .build();


        firstPickup = follower.pathBuilder()
                .addPath(new BezierLine(new Point(score), new Point(sample1)))
                .setLinearHeadingInterpolation(score.getHeading(), sample1.getHeading())
                .addParametricCallback(0.1, ()->setSliderMinPosition())
                .addParametricCallback(0.5, () -> follower.setMaxPower(0.6))
                .setZeroPowerAccelerationMultiplier(4.0)
                .setPathEndTimeoutConstraint(0)
                .build();

        score1 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(sample1), new Point(score)))
                .setLinearHeadingInterpolation(sample1.getHeading(), score.getHeading())
                .setPathEndTimeoutConstraint(0)
                .build();

        secondPickup = follower.pathBuilder()
                .addPath(new BezierLine(new Point(score), new Point(sample2)))
                .setLinearHeadingInterpolation(score.getHeading(), sample2.getHeading())
                .addParametricCallback(0.1, ()->setSliderMinPosition())
                .addParametricCallback(0.5, () -> follower.setMaxPower(0.6))
                .setZeroPowerAccelerationMultiplier(4.0)
                .setPathEndTimeoutConstraint(0)
                .build();

        score2 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(sample2), new Point(score)))
                .setLinearHeadingInterpolation(sample2.getHeading(), score.getHeading())
                .setPathEndTimeoutConstraint(0)
                .build();

        thirdPickup = follower.pathBuilder()
                .addPath(new BezierLine(new Point(score), new Point(sample3)))
                .setLinearHeadingInterpolation(score.getHeading(), sample3.getHeading())
                .addParametricCallback(0.1, ()->setSliderMinPosition())
                .addParametricCallback(0.5, () -> follower.setMaxPower(0.7))
                .setZeroPowerAccelerationMultiplier(4.0)
                .setPathEndTimeoutConstraint(0)
                .build();

        score3 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(sample3), new Point(score)))
                .setLinearHeadingInterpolation(sample3.getHeading(), score.getHeading())
                .build();

        subPath = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(score), new Point(subControl), new Point(sub1)))
                .addParametricCallback(0, ()-> follower.setMaxPower(0.9))
                .setLinearHeadingInterpolation(score.getHeading(), sub1.getHeading())
                .addParametricCallback(0, ()->setSliderMinPosition())
                .setZeroPowerAccelerationMultiplier(4.0)
                .build();
    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                targetPosition = OuttakeConstants.OUTTAKE_MAX_POSITION;
                follower.followPath(scorePreload);
                setPathState(1);
                break;

            case 1:
                if(!follower.isBusy()){
                    scoreSample(0.6);
                    if(hasScored) {
                        follower.followPath(firstPickup);
                        startHoverTimer = true;
                        setPathState(2);
                    }
                }
                break;
            case 2:
                if(!follower.isBusy()){
                    extendAndPickUp(0.4, ServoConstants.INTAKE_WRIST_ROT_0_DEGREES);
                }
                if(hasFinishedPickUp) {
                    follower.followPath(score1);
                    setPathState(3);
                }
                break;
            case 3:
                if(hasFinishedPickUp && !hasTransfered){
                    transfer(0.6);
                }
                if(hasTransfered && robotConfig.upMotor.getCurrentPosition() > targetPosition - 50)
                    scoreSample(0.6);
                if(hasScored) {
                    follower.followPath(secondPickup);
                    setPathState(4);
                }
                break;

            //sample 3, to be tested
            case 4:
                if(!follower.isBusy()){
                    extendAndPickUp(0.4,ServoConstants.INTAKE_WRIST_ROT_0_DEGREES);
                }
                if(hasFinishedPickUp) {
                    follower.followPath(score2);
                    setPathState(5);
                }
                break;
            case 5:
                if(hasFinishedPickUp && !hasTransfered){
                    transfer(0.6);
                }
                if(hasTransfered && robotConfig.upMotor.getCurrentPosition() > targetPosition - 50)
                    scoreSample(0.6);
                if(hasScored) {
                    follower.followPath(thirdPickup);
                    setPathState(6);
                }
                break;
            case 6:
                if(!follower.isBusy()){
                    extendAndPickUp(0.5, .05);
                }
                if(hasFinishedPickUp) {
                    follower.followPath(score3);
                    setPathState(7);
                }
                break;
            case 7:
                if(hasFinishedPickUp && !hasTransfered){
                    transfer(0.6);
                  }
                if(hasTransfered && robotConfig.upMotor.getCurrentPosition() > targetPosition - 50)
                    scoreSample(0.6);
                if(hasScored) {
                    follower.setMaxPower(0.9);
                    follower.followPath(subPath);
                    setPathState(-1);
                }
                 break;
        }
    }

    /**
     * These change the states of the paths and actions
     * It will also reset the timers of the individual switches
     **/
    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }

    @Override
    public void init() {
        hasScored = false;
        hasTransfered = false;
        hasFinishedPickUp = false;
        robotConfig = new RobotConfig(hardwareMap);
        robotConfig.upMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robotConfig.midMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robotConfig.downMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        robotConfig.upMotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        robotConfig.midMotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        robotConfig.downMotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);


        robotConfig.upMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        robotConfig.midMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        robotConfig.downMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);


        updatePIDFController();
        pathTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();

        Constants.setConstants(FConstants.class, LConstants.class);
        follower = new Follower(hardwareMap);
        follower.setStartingPose(startPose);
//        initializeCamera();
        resetServosToInit();
        buildPaths();

    }

    /**
     * This is the main loop of the OpMode, it will run repeatedly after clicking "Play".
     **/
    @Override
    public void loop() {

        follower.update();
//        processCamera();
        autonomousPathUpdate();
        controlPID();
        telemetry.addData("da",da);
        telemetry.addData("numarare", numarare);
        telemetry.addData("retractTimer", retractTimer.seconds());
        telemetry.addData("hasScored", hasScored);
        telemetry.addData("hasTrans", hasTransfered);
        telemetry.addData("Target Position", targetPosition);
        telemetry.addData("Left Motor Position", robotConfig.upMotor.getCurrentPosition());
        telemetry.addData("Right Motor Position", robotConfig.downMotor.getCurrentPosition());
        telemetry.addData("Left Motor Error", robotConfig.upMotor.getCurrentPosition() - targetPosition);
        telemetry.addData("Right Motor Error", robotConfig.downMotor.getCurrentPosition() - targetPosition);
        telemetry.addData("case", pathState);
        telemetry.addData("intakeTimer", intakeTimer.seconds());
        telemetry.update();
    }

    /**
     * This method is called continuously after Init while waiting for "play".
     **/
    @Override
    public void init_loop() {
    }

    /**
     * This method is called once at the start of the OpMode.
     * It runs all the setup actions, including building paths and starting the path system
     **/
    @Override
    public void start() {
        opmodeTimer.resetTimer();
        setPathState(0);
    }

    /**
     * We do not use this because everything should automatically disable
     **/
    @Override
    public void stop() {
    }

    private void updatePIDFController() {
        CustomPIDFCoefficients coefficients = new CustomPIDFCoefficients(P, I, D, F);
        pidfController = new PIDFController(coefficients);
    }

    private void controlPID() {
        pidfController.setTargetPosition(targetPosition);
        pidfController.updatePosition(robotConfig.upMotor.getCurrentPosition());

        double powerUp = pidfController.runPIDF() + K;

        if (Math.abs(robotConfig.upMotor.getCurrentPosition() - targetPosition) <= OuttakeConstants.TOLERANCE) {
            powerUp = 0;
        }
        if (robotConfig.upMotor.getCurrentPosition() > 10)
            areSlidesDown = false;
        if (robotConfig.upMotor.getCurrentPosition() < 100 && robotConfig.upMotor.getVelocity() < 0.05 && targetPosition == OuttakeConstants.OUTTAKE_MIN_POSITION) {
            RobotConfig.upMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
            RobotConfig.midMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
            RobotConfig.downMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

            RobotConfig.upMotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
            RobotConfig.midMotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
            RobotConfig.downMotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
            areSlidesDown = true;
        }
        if (targetPosition == OuttakeConstants.OUTTAKE_MIN_POSITION && areSlidesDown) {
            robotConfig.upMotor.setPower(0);
            robotConfig.midMotor.setPower(0);
            robotConfig.downMotor.setPower(0);
        }
        if (targetPosition == OuttakeConstants.OUTTAKE_MIN_POSITION && robotConfig.upMotor.getCurrentPosition() > 10) {
            robotConfig.upMotor.setPower(-1);
            robotConfig.midMotor.setPower(-1);
            robotConfig.downMotor.setPower(-1);

        } else {
            robotConfig.upMotor.setPower(powerUp);
            robotConfig.midMotor.setPower(powerUp);
            robotConfig.downMotor.setPower(powerUp);
        }
        robotConfig.upMotor.setPower(powerUp);
        robotConfig.midMotor.setPower(powerUp);
        robotConfig.downMotor.setPower(powerUp);
    }

    private void resetServosToInit() {
        robotConfig.setOuttakeServoPositions(
                ServoConstants.OUTTAKE_CLAW_CLOSED_POSITION,
                ServoConstants.OUTTAKE_WRIST_ROT_180_DEGREES,
                ServoConstants.OUTTAKE_WRIST_Y_TRANSFER_POSITION,
                ServoConstants.OUTTAKE_ELBOW_RIGHT_PICKUP_POSITION,
                ServoConstants.OUTTAKE_ELBOW_LEFT_PICKUP_POSITION
        );
        robotConfig.setIntakeServoPositions(
                ServoConstants.INTAKE_ELBOW_RIGHT_RETRACTED_POSITION,
                ServoConstants.INTAKE_ELBOW_LEFT_RETRACTED_POSITION,
                ServoConstants.INTAKE_WRIST_MID,
                ServoConstants.INTAKE_WRIST_RIGHT_UP_POSITION,
                ServoConstants.INTAKE_WRIST_LEFT_UP_POSITION,
                ServoConstants.INTAKE_CLAW_OPEN_POSITION,
                ServoConstants.INTAKE_WRIST_ROT_0_DEGREES
        );
    }

    private void openExtend(double servoPos) {
        robotConfig.setIntakeServoPositions(
                ServoConstants.INTAKE_ELBOW_RIGHT_EXTENDED_POSITION,
                ServoConstants.INTAKE_ELBOW_LEFT_EXTENDED_POSITION,
                ServoConstants.INTAKE_WRIST_DOWN,
                ServoConstants.INTAKE_WRIST_RIGHT_DOWN_POSITION,
                ServoConstants.INTAKE_WRIST_LEFT_DOWN_POSITION,
                ServoConstants.INTAKE_CLAW_OPEN_POSITION,
                servoPos);
    }

    private void closeExtend(double servoPos) {
        robotConfig.setIntakeServoPositions(
                ServoConstants.INTAKE_ELBOW_RIGHT_EXTENDED_POSITION,
                ServoConstants.INTAKE_ELBOW_LEFT_EXTENDED_POSITION,
                ServoConstants.INTAKE_WRIST_DOWN,
                ServoConstants.INTAKE_WRIST_RIGHT_DOWN_POSITION,
                ServoConstants.INTAKE_WRIST_LEFT_DOWN_POSITION,
                ServoConstants.INTAKE_CLAW_CLOSED_POSITION,
                servoPos
        );
    }

    private void hoverExtend(double servoPos) {
        robotConfig.setIntakeServoPositions(
                ServoConstants.INTAKE_ELBOW_RIGHT_EXTENDED_POSITION,
                ServoConstants.INTAKE_ELBOW_LEFT_EXTENDED_POSITION,
                ServoConstants.INTAKE_WRIST_DOWN,
                ServoConstants.INTAKE_WRIST_RIGHT_HOVER_POSITION,
                ServoConstants.INTAKE_WRIST_LEFT_HOVER_POSITION,
                ServoConstants.INTAKE_CLAW_OPEN_POSITION,
                servoPos
        );
    }

    private VisionPortal visionPortal;
    private SampleOrientationProcessor processor;

    private void initializeCamera() {
        processor = new SampleOrientationProcessor(telemetry);
        visionPortal = VisionPortal.easyCreateWithDefaults(
                hardwareMap.get(WebcamName.class, "camera"), // Assuming "camera" is the webcam name
                processor
        );
    }

    private void processCamera() {
        // Process camera data and use the detected sample angle
        double sampleAngle = processor.getSampleAngle();
        double sampleAngleDegrees = sampleAngle * 180 / Math.PI;

        // Adjust servo position based on sample angle
        double servoPosition = getServoPositionFromAngle(sampleAngleDegrees);
        robotConfig.intakeWristRotServo.setPosition(servoPosition);

        ArrayList<org.opencv.core.Point> detectedObjects = processor.getOffsets();
        if (!detectedObjects.isEmpty()) {
            org.opencv.core.Point largestObject = detectedObjects.get(0);
            for (org.opencv.core.Point obj : detectedObjects) {
                if (obj.y > largestObject.y) { // Larger object detection logic
                    largestObject = obj;
                }
            }
            telemetry.addData("Tracking Object", largestObject);
        }

//        telemetry.addData("Sample Angle (radians)", sampleAngle);
//        telemetry.addData("Sample Angle (degrees)", sampleAngleDegrees);
//        telemetry.addData("Offsets", processor.getOffsets().toString());
//        telemetry.update();
    }

    // Use sample angle to determine the servo position
    private double getServoPositionFromAngle(double angle) {
        if (angle > -30 && angle <= 30) {
            return 0.29;
        } else if (angle > 30 && angle <= 60) {
            return 0.56;
        } else if ((angle > 60 && angle <= 90) || (angle <= -60 && angle >= -90)) {
            return 0.84;
        } else if (angle <= -30 && angle > -60) {
            return 0.02;
        } else {
            return 0.29;  // Default case (to avoid undefined behavior)
        }
    }

    private void scoreSample(double timer){
        if(startTimer){
            startHoverTimer = true;
            retractTimer.reset();
            hasFinishedPickUp = false;
            startTimer = false;
        }
        targetPosition = OuttakeConstants.OUTTAKE_MAX_POSITION;
        if (robotConfig.upMotor.getCurrentPosition() > targetPosition - 50 && retractTimer.seconds()<timer) {
            robotConfig.outtakeElbowRightServo.setPosition(ServoConstants.OUTTAKE_ELBOW_RIGHT_PLACE_SAMPLE_POSITION);
            robotConfig.outtakeElbowLeftServo.setPosition(ServoConstants.OUTTAKE_ELBOW_LEFT_PLACE_SAMPLE_POSITION);
            robotConfig.outtakeWristYServo.setPosition(ServoConstants.OUTTAKE_WRIST_Y_PLACE_POSITION);
        }
        if (retractTimer.seconds() > timer) { // 1.2
            robotConfig.outtakeClawServo.setPosition(ServoConstants.OUTTAKE_CLAW_OPEN_POSITION);
            if (robotConfig.outtakeClawServo.getPosition() == ServoConstants.OUTTAKE_CLAW_OPEN_POSITION) {
                if (retractTimer.seconds() > timer + 0.3) { // 1.5
                    robotConfig.outtakeElbowRightServo.setPosition(ServoConstants.OUTTAKE_ELBOW_RIGHT_PICKUP_POSITION);
                    robotConfig.outtakeElbowLeftServo.setPosition(ServoConstants.OUTTAKE_ELBOW_LEFT_PICKUP_POSITION);
                    robotConfig.outtakeWristYServo.setPosition(ServoConstants.OUTTAKE_WRIST_Y_TRANSFER_POSITION);
                    if (retractTimer.seconds() > timer + 0.6) { // 1.8
                            hasScored = true;

                    }
                }

            }
        }
    }
    private void setSliderMinPosition(){
        targetPosition = OuttakeConstants.OUTTAKE_MIN_POSITION;
    }
    private void transfer(double timer) {
        if(startTransferTimer){
            intakeTimer.reset();
            startTransferTimer = false;
            startTimer = true;
            transfer = false;
            closed = false;
            hasScored = false;
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
        if (transfer && intakeTimer.seconds() > timer) {
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
            if (intakeTimer.seconds() >timer + 0.1) {
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
        if(hasTransfered )
            targetPosition = OuttakeConstants.OUTTAKE_MAX_POSITION;

    }

    private void extendAndPickUp(double timer, double servoPos){ //true
        if(startHoverTimer){
            startTransferTimer = true;
            hoverTimer.reset();
            hasTransfered = false;
            startHoverTimer = false;
            intaking = true;
        }
        if(intaking){
            hoverExtend(servoPos);
            intaking = false;
        }
        if (hoverTimer.seconds() > timer) {
            intaking = false;
            if (!intaking && hoverTimer.seconds() > timer + 0.1 && hoverTimer.seconds()<timer + 0.25){ // 2 s
                openExtend(servoPos);
            }
            if (hoverTimer.seconds() > timer + 0.25) {
                closeExtend(servoPos);
                if (robotConfig.intakeClawServo.getPosition() > ServoConstants.INTAKE_CLAW_CLOSED_POSITION - 0.05) {
                    hasFinishedPickUp = true;
                }
            }
        }
    }
}