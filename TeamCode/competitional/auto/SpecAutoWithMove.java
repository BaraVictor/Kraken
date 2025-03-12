package org.firstinspires.ftc.teamcode.competitional.auto;

import static java.lang.Math.toRadians;
import static java.lang.Thread.sleep;

import com.acmerobotics.dashboard.FtcDashboard;
import com.pedropathing.util.Constants;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import  com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.configurations.RobotConfig;
import com.pedropathing.follower.*;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierCurve;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;
import com.pedropathing.util.Timer;
import com.pedropathing.util.CustomPIDFCoefficients;
import com.pedropathing.util.PIDFController;

import org.firstinspires.ftc.teamcode.constants.OuttakeConstants;
import org.firstinspires.ftc.teamcode.constants.ServoConstants;
import org.firstinspires.ftc.teamcode.cvProcessors.SampleOrientationProcessor;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.FConstants;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.LConstants;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;

import java.util.ArrayList;
import java.util.List;

//@Config
@Autonomous(name = "🐙 SampleAuto (4+0) + move 🐙", group = "A. Competitional")
public class SpecAutoWithMove extends OpMode {
    private VisionPortal visionPortal;
    private SampleOrientationProcessor processor;

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
    private double prevY;

    private static final double MIN_POSITION = 0.22;  // Fully retracted
    private static final double MAX_POSITION = 0.53;  // Fully extended
    private static final double EXTEND_STEP_SIZE2 = 0.006;
    private static final double X_MIN = -3;
    private static final double X_MAX = 0.5;
    public double retract = 0.07;// Smaller step size to ensure smoother movement (smaller change)
    double currentPosition = MIN_POSITION;
    double sampleY;
    double sampleX;
    double sampleAngle;
    boolean sampleFound = false;
    boolean canpickUp = false;
    int sampleId;
    boolean canDown = false;
    public ElapsedTime pickupTimer = new ElapsedTime();
    public ElapsedTime cameraTimer = new ElapsedTime();

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

    private final Pose scorePre = new Pose(19, 133, toRadians(315));

    private final Pose sample1 = new Pose(28.5, 129, toRadians(0));

    private final Pose sample2 = new Pose(28.5, 138.5, toRadians(0));

    private final Pose sample3 = new Pose(33.5, 138, toRadians(45));

    private int sub1X = 63;

    private int sub2X = 66;

    private final Pose sub1 = new Pose(sub1X, 92, Math.toRadians(-90));

    private final Pose subLeft = new Pose(64.3, 92, Math.toRadians(-90));

    private final Pose subRight = new Pose(61.7, 92, Math.toRadians(-90));


    private final Pose sub2 = new Pose(sub2X, 91, Math.toRadians(-90));

    private final Pose sub2Left = new Pose(67.3,91,Math.toRadians(-90));

    private final Pose sub2Right = new Pose(64.7,91,Math.toRadians(-90));



    private final Pose subControl = new Pose(65, 118, Math.toRadians(-90));

    private final Pose parkFinal = new Pose(75, 93, Math.toRadians(90));


    private PathChain subPath2, swipeLeft2,swipeRight2,return2,swipeLeft, swipeRight,return1, scorePreload, firstPickup, secondPickup, thirdPickup, subPath, score1, score2, score3; //?

    public void buildPaths() {

        /* This is our grabPickup1 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        scorePreload = follower.pathBuilder()
                .addPath(new BezierLine(new Point(startPose), new Point(scorePre)))
                .setLinearHeadingInterpolation(startPose.getHeading(), scorePre.getHeading())
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
                .addParametricCallback(0, () -> follower.setMaxPower(1))
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
                .addParametricCallback(0.8, ()-> follower.setMaxPower(0.6))
                .setLinearHeadingInterpolation(score.getHeading(), sub1.getHeading())
                .addParametricCallback(0, ()->setSliderMinPosition())
                .setZeroPowerAccelerationMultiplier(2.0)
                .build();

        return1 = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(sub1), new Point(subControl), new Point(score)))
                .setLinearHeadingInterpolation(sub1.getHeading(), score.getHeading())
                .addParametricCallback(0, ()-> follower.setMaxPower(1))
                .setZeroPowerAccelerationMultiplier(4.0)
                .addParametricCallback(0.65,()->liftSlides())
                .build();

        swipeLeft = follower.pathBuilder()
                .addPath(new BezierLine(new Point(sub1), new Point(subLeft)))
                .setLinearHeadingInterpolation(sub1.getHeading(), subLeft.getHeading())
                .build();
        swipeRight = follower.pathBuilder()
                .addPath(new BezierLine(new Point(sub1), new Point(subRight)))
                .setLinearHeadingInterpolation(sub1.getHeading(), subRight.getHeading())
                .build();
        subPath2 = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(score),new Point(subControl),new Point(sub2)))
                .addParametricCallback(0, ()-> follower.setMaxPower(1))
                .addParametricCallback(0.8, ()-> follower.setMaxPower(0.6))
                .setLinearHeadingInterpolation(score.getHeading(), sub2.getHeading())
                .addParametricCallback(0, ()->setSliderMinPosition())
                .setZeroPowerAccelerationMultiplier(2.0)
                .build();
        swipeLeft2 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(sub2), new Point(sub2Left)))
                .setLinearHeadingInterpolation(sub2.getHeading(), sub2Left.getHeading())
                .build();
        swipeRight2 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(sub2), new Point(sub2Right)))
                .setLinearHeadingInterpolation(sub2.getHeading(), sub2Right.getHeading())
                .build();
        return2 = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(sub2), new Point(subControl), new Point(score)))
                .setLinearHeadingInterpolation(sub2.getHeading(), score.getHeading())
                .addParametricCallback(0, ()-> follower.setMaxPower(1))
                .setZeroPowerAccelerationMultiplier(4.0)
                .addParametricCallback(0.65,()->liftSlides())
                .build();

    }

    public void autonomousPathUpdate() throws InterruptedException {
        switch (pathState) {

            case 0:
//                targetPosition = OuttakeConstants.OUTTAKE_MAX_POSITION;
//                robotConfig.outtakeElbowLeftServo.setPosition(ServoConstants.OUTTAKE_ELBOW_LEFT_PLACE_SAMPLE_VERTICAL);
//                robotConfig.outtakeElbowRightServo.setPosition(ServoConstants.OUTTAKE_ELBOW_RIGHT_PLACE_SAMPLE_VERTICAL);
//                follower.followPath(scorePreload);
                cameraTimer.reset();
                setPathState(8);
                break;

            case 1:
                if(!follower.isBusy()){
                    scoreSample(0.4);
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
                    transfer(0.6, true);
                }
                if(hasTransfered && robotConfig.upMotor.getCurrentPosition() > targetPosition - 70)
                    scoreSample(0.4);
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
                    transfer(0.6, true);
                }
                if(hasTransfered  && robotConfig.upMotor.getCurrentPosition() > targetPosition - 70)
                    scoreSample(0.4);
                if(hasScored) {
                    follower.followPath(thirdPickup);
                    setPathState(6);
                }
                break;
            case 6:
                if(!follower.isBusy()){
                    extendAndPickUp(0.5, ServoConstants.INTAKE_WRIST_ROT_minus_45_DEGREES);
                }
                if(hasFinishedPickUp) {
                    follower.followPath(score3);
                    setPathState(7);
                }
                break;
            case 7:
                if(hasFinishedPickUp && !hasTransfered){
                    transfer(0.6, true);
                }
                if(hasTransfered  && robotConfig.upMotor.getCurrentPosition() > targetPosition - 70)
                    scoreSample(0.4);
                if(hasScored) {
                    follower.setMaxPower(0.9);
                    follower.followPath(subPath);
                    setPathState(8);
                }
                break;
            case 8:
                if(!follower.isBusy()) {
                    robotConfig.intakeWristServo.setPosition(ServoConstants.INTAKE_WRIST_PERPENDICULAR);
                    robotConfig.intakeWristRightServo.setPosition(ServoConstants.INTAKE_WRIST_RIGHT_CAMERA_POSITION);
                    robotConfig.intakeWristLeftServo.setPosition(ServoConstants.INTAKE_WRIST_LEFT_CAMERA_POSITION);
                    cameraTimer.reset();
                    setPathState(9);
                }
                break;

            case 9:
                if(!follower.isBusy() && !sampleFound) {
                    robotConfig.intakeWristRightServo.setPosition(ServoConstants.INTAKE_WRIST_RIGHT_CAMERA_POSITION);
                    robotConfig.intakeWristLeftServo.setPosition(ServoConstants.INTAKE_WRIST_LEFT_CAMERA_POSITION);
                    if (cameraTimer.seconds() > 0.5 && !sampleFound ) {
                        ArrayList<double[]> samples = processor.getRealPositions();
                        for (int i = 0; i < samples.size(); i++) {
                                sampleX = samples.get(i)[0];  // X position of the sample
                                sampleY = samples.get(i)[1];
                            if (sampleX <= X_MIN && sampleY <= 0.5) {
                                sampleId = i;
                                sampleFound = true;
                                follower.followPath(swipeLeft);
                               // setPathState(10);
                            }
                                if (sampleX > X_MIN && sampleX < X_MAX && sampleY <=0.5) {
                                    sampleFound = true;
                                    sampleId = i;
                                 //   setPathState(12);
                                }
                            if (sampleX >= X_MAX && sampleY <= 0.5) {
                                sampleFound = true;
                                sampleId = i;
                                follower.followPath(swipeRight);
                               // setPathState(14);
                            }
                        }
                        if (!sampleFound) {
                            // Gradually extend the arm until a sample is found
                            extendArmGradually();
                        }
                    }
                }
                if(sampleFound && !follower.isBusy()){
                    if(!canpickUp) {
                        sampleAngle = mapAngleToServoPosition(processor.getSampleAngles().get(sampleId));
                        currentPosition = robotConfig.intakeElbowLeftServo.getPosition();
                        robotConfig.outtakeClawServo.setPosition(ServoConstants.OUTTAKE_CLAW_OPEN_POSITION);
                        robotConfig.intakeWristRotServo.setPosition(mapAngleToServoPosition(sampleAngle));
                        canpickUp = true;
                        pickupTimer.reset();
                    }
                    if(pickupTimer.seconds()>0.7 && canpickUp && !canDown){
                        robotConfig.intakeWristLeftServo.setPosition(ServoConstants.INTAKE_WRIST_LEFT_HOVER_POSITION);
                        robotConfig.intakeWristRightServo.setPosition(ServoConstants.INTAKE_WRIST_RIGHT_HOVER_POSITION);
                        robotConfig.intakeWristServo.setPosition(ServoConstants.INTAKE_WRIST_DOWN);
                        robotConfig.intakeElbowLeftServo.setPosition(currentPosition - retract);
                        robotConfig.intakeElbowRightServo.setPosition(currentPosition - retract);
                        canDown = true;
                    }
                    if(pickupTimer.seconds()>0.9 && canpickUp){
                        robotConfig.intakeWristRightServo.setPosition(ServoConstants.INTAKE_WRIST_RIGHT_DOWN_POSITION+0.03);
                        robotConfig.intakeWristLeftServo.setPosition(ServoConstants.INTAKE_WRIST_LEFT_DOWN_POSITION+0.03);
                    }
                    if(pickupTimer.seconds()>1.1 && canpickUp){
                        robotConfig.intakeClawServo.setPosition(ServoConstants.INTAKE_CLAW_CLOSED_POSITION);
                    }
                    if(pickupTimer.seconds()>2){
                        startTransferTimer = true;
                        hasTransfered = false;
                       // follower.followPath(return1);
                       // setPathState(11);
                    }

                }
                break;
            case 10:
                if(!follower.isBusy()) {
                    if(!canpickUp) {
                        currentPosition = robotConfig.intakeElbowLeftServo.getPosition();
                        robotConfig.outtakeClawServo.setPosition(ServoConstants.OUTTAKE_CLAW_OPEN_POSITION);
                        robotConfig.intakeElbowLeftServo.setPosition(currentPosition - retract);
                        robotConfig.intakeElbowRightServo.setPosition(currentPosition - retract);
                        robotConfig.intakeWristLeftServo.setPosition(ServoConstants.INTAKE_WRIST_LEFT_HOVER_POSITION);
                        robotConfig.intakeWristRightServo.setPosition(ServoConstants.INTAKE_WRIST_RIGHT_HOVER_POSITION);
                        robotConfig.intakeWristServo.setPosition(ServoConstants.INTAKE_WRIST_DOWN);
                        robotConfig.intakeWristRotServo.setPosition(mapAngleToServoPosition(sampleAngle));
                        canpickUp = true;
                        pickupTimer.reset();
                    }
                    if(pickupTimer.seconds()>0.5 && canpickUp){
                        robotConfig.intakeWristRightServo.setPosition(ServoConstants.INTAKE_WRIST_RIGHT_DOWN_POSITION+0.03);
                        robotConfig.intakeWristLeftServo.setPosition(ServoConstants.INTAKE_WRIST_LEFT_DOWN_POSITION+0.03);
                    }
                    if(pickupTimer.seconds()>0.8 && canpickUp){
                        robotConfig.intakeClawServo.setPosition(ServoConstants.INTAKE_CLAW_CLOSED_POSITION);
                    }
                    if(pickupTimer.seconds()>1.1){
                        startTransferTimer = true;
                        hasTransfered = false;
                        follower.followPath(return1);
                        setPathState(11);
                    }
                }
                break;
            case 11:
                transfer(0.6, false);
                if(hasTransfered){
                 setPathState(16);
                 }
                break;
            case 12:
                if(!canpickUp) {
                    currentPosition = robotConfig.intakeElbowLeftServo.getPosition();
                    robotConfig.intakeClawServo.setPosition(ServoConstants.INTAKE_CLAW_OPEN_POSITION);
                    robotConfig.outtakeClawServo.setPosition(ServoConstants.OUTTAKE_CLAW_OPEN_POSITION);
                    robotConfig.intakeWristLeftServo.setPosition(ServoConstants.INTAKE_WRIST_LEFT_HOVER_POSITION);
                    robotConfig.intakeWristRightServo.setPosition(ServoConstants.INTAKE_WRIST_RIGHT_HOVER_POSITION);
                    robotConfig.intakeElbowLeftServo.setPosition(currentPosition - retract);
                    robotConfig.intakeElbowRightServo.setPosition(currentPosition - retract);
                    robotConfig.intakeWristServo.setPosition(ServoConstants.INTAKE_WRIST_DOWN);
                    robotConfig.intakeWristRotServo.setPosition(mapAngleToServoPosition(sampleAngle));
                    canpickUp = true;
                    pickupTimer.reset();
                }
                if(pickupTimer.seconds()>0.5 && canpickUp){
                    robotConfig.intakeWristRightServo.setPosition(ServoConstants.INTAKE_WRIST_RIGHT_DOWN_POSITION+0.03);
                    robotConfig.intakeWristLeftServo.setPosition(ServoConstants.INTAKE_WRIST_LEFT_DOWN_POSITION+0.03);
                }
                if(pickupTimer.seconds()>0.8 && canpickUp){
                    robotConfig.intakeClawServo.setPosition(ServoConstants.INTAKE_CLAW_CLOSED_POSITION);
                }
                if(pickupTimer.seconds()>1.1){
                    startTransferTimer = true;
                    hasTransfered = false;
                    follower.followPath(return1);
                    setPathState(13);
                }
                break;
            case 13:
                transfer(0.6, false);
                if(hasTransfered){
                    setPathState(16);
                }
                break;
            case 14:
                if(!follower.isBusy()) {
                    if(!canpickUp) {
                        currentPosition = robotConfig.intakeElbowLeftServo.getPosition();
                        List<double[]> newSamples = processor.getRealPositions();
                        List<Double> newAngles = processor.getSampleAngles();

                        for (int i = 0; i < newSamples.size(); i++) {
                            double newY = newSamples.get(i)[1];
                            // Check if this sample is the same as the previously detected one
                            if (newY<=1 && newY>=-1) {
                                sampleAngle = newAngles.get(i);

                            }
                        }
                        robotConfig.outtakeClawServo.setPosition(ServoConstants.OUTTAKE_CLAW_OPEN_POSITION);
                        robotConfig.intakeWristLeftServo.setPosition(ServoConstants.INTAKE_WRIST_LEFT_HOVER_POSITION);
                        robotConfig.intakeWristRightServo.setPosition(ServoConstants.INTAKE_WRIST_RIGHT_HOVER_POSITION);
                        robotConfig.intakeElbowLeftServo.setPosition(currentPosition - retract);
                        robotConfig.intakeElbowRightServo.setPosition(currentPosition - retract);
                        robotConfig.intakeWristServo.setPosition(ServoConstants.INTAKE_WRIST_DOWN);
                        robotConfig.intakeWristRotServo.setPosition(mapAngleToServoPosition(sampleAngle));
                        canpickUp = true;
                        pickupTimer.reset();
                    }
                    if(pickupTimer.seconds()>0.5 && canpickUp){
                        robotConfig.intakeWristRightServo.setPosition(ServoConstants.INTAKE_WRIST_RIGHT_DOWN_POSITION+0.03);
                        robotConfig.intakeWristLeftServo.setPosition(ServoConstants.INTAKE_WRIST_LEFT_DOWN_POSITION+0.03);
                    }
                    if(pickupTimer.seconds()>0.8 && canpickUp){
                        robotConfig.intakeClawServo.setPosition(ServoConstants.INTAKE_CLAW_CLOSED_POSITION);
                    }
                    if(pickupTimer.seconds()>1.1){
                        startTransferTimer = true;
                        hasTransfered = false;
                        follower.followPath(return1);
                        setPathState(15);
                    }
                }
                break;
            case 15:
                transfer(0.6, false);
                  if(hasTransfered){
                   setPathState(16);
                  }
                break;
            case 16:
                if(!follower.isBusy()){
                    scoreSample(0.4);
                }
                if(hasScored) {
                    follower.followPath(subPath2);
                    setPathState(17);
                }
                break;
            case 17:
                if(!follower.isBusy()) {
                    robotConfig.intakeWristRightServo.setPosition(ServoConstants.INTAKE_WRIST_RIGHT_CAMERA_POSITION);
                    robotConfig.intakeWristLeftServo.setPosition(ServoConstants.INTAKE_WRIST_LEFT_CAMERA_POSITION);
                    currentPosition = MIN_POSITION;
                    sampleFound = false;
                    cameraTimer.reset();
                    setPathState(18);
                }
            case 18:
                if(!follower.isBusy()) {
                    robotConfig.intakeWristRightServo.setPosition(ServoConstants.INTAKE_WRIST_RIGHT_CAMERA_POSITION);
                    robotConfig.intakeWristLeftServo.setPosition(ServoConstants.INTAKE_WRIST_LEFT_CAMERA_POSITION);
                    if (cameraTimer.seconds() > 0.3) {
                        ArrayList<double[]> samples = processor.getRealPositions();
                        ArrayList<Double> sampleAngles = processor.getSampleAngles();
                        for (int i = 0; i < samples.size(); i++) {
                            if (i < sampleAngles.size()) {
                                sampleX = samples.get(i)[0];  // X position of the sample
                                sampleY = samples.get(i)[1];
                                if (sampleX <= X_MIN && sampleY <= 0) {
                                    sampleId = i;
                                    follower.followPath(swipeLeft2);
                                    canpickUp = false;
//                                    setPathState(19);
                                }
                                if (sampleX > X_MIN && sampleX < X_MAX && sampleY <= 1) {
                                    sampleId = i;
                                    canpickUp = false;
//                                    setPathState(20);
                                }
                                if (sampleX >= X_MAX && sampleY <= 0) {
                                    sampleId = i;
                                    follower.followPath(swipeRight2);
                                    canpickUp = false;
//                                    setPathState(22);
                                }
                            }
                        }
                        if (!sampleFound) {
                            // Gradually extend the arm until a sample is found
                            extendArmGradually();
                        }
                    }
                }
                if(sampleFound && !follower.isBusy()){
                    if(!canpickUp) {
                        sampleAngle = mapAngleToServoPosition(processor.getSampleAngles().get(sampleId));
                        currentPosition = robotConfig.intakeElbowLeftServo.getPosition();
                        robotConfig.outtakeClawServo.setPosition(ServoConstants.OUTTAKE_CLAW_OPEN_POSITION);
                        robotConfig.intakeWristLeftServo.setPosition(ServoConstants.INTAKE_WRIST_LEFT_HOVER_POSITION);
                        robotConfig.intakeWristRightServo.setPosition(ServoConstants.INTAKE_WRIST_RIGHT_HOVER_POSITION);
                        robotConfig.intakeWristServo.setPosition(ServoConstants.INTAKE_WRIST_DOWN);
                        robotConfig.intakeWristRotServo.setPosition(mapAngleToServoPosition(sampleAngle));
                        canpickUp = true;
                        pickupTimer.reset();
                    }
                    if(pickupTimer.seconds()>0.5 && canpickUp){
                        robotConfig.intakeElbowLeftServo.setPosition(currentPosition - retract);
                        robotConfig.intakeElbowRightServo.setPosition(currentPosition - retract);
                        robotConfig.intakeWristRightServo.setPosition(ServoConstants.INTAKE_WRIST_RIGHT_DOWN_POSITION+0.03);
                        robotConfig.intakeWristLeftServo.setPosition(ServoConstants.INTAKE_WRIST_LEFT_DOWN_POSITION+0.03);
                    }
                    if(pickupTimer.seconds()>0.8 && canpickUp){
                        robotConfig.intakeClawServo.setPosition(ServoConstants.INTAKE_CLAW_CLOSED_POSITION);
                    }
                    if(pickupTimer.seconds()>1.1){
                        startTransferTimer = true;
                        hasTransfered = false;
                        follower.followPath(return1);
                        setPathState(20);
                    }
                }
                break;
            case 19:
                if(!follower.isBusy()) {
                    if(!canpickUp) {
                        currentPosition = robotConfig.intakeElbowLeftServo.getPosition();
                        List<double[]> newSamples = processor.getRealPositions();
                        List<Double> newAngles = processor.getSampleAngles();

                        for (int i = 0; i < newSamples.size(); i++) {
                            double newY = newSamples.get(i)[1];
                            // Check if this sample is the same as the previously detected one
                            if (newY<=1 && newY>=-1) {
                                sampleAngle = newAngles.get(i);

                            }
                        }
                        robotConfig.outtakeClawServo.setPosition(ServoConstants.OUTTAKE_CLAW_OPEN_POSITION);
                        robotConfig.intakeWristLeftServo.setPosition(ServoConstants.INTAKE_WRIST_LEFT_HOVER_POSITION);
                        robotConfig.intakeWristRightServo.setPosition(ServoConstants.INTAKE_WRIST_RIGHT_HOVER_POSITION);
                        robotConfig.intakeElbowLeftServo.setPosition(currentPosition - retract);
                        robotConfig.intakeElbowRightServo.setPosition(currentPosition - retract);
                        robotConfig.intakeWristServo.setPosition(ServoConstants.INTAKE_WRIST_DOWN);
                        robotConfig.intakeWristRotServo.setPosition(mapAngleToServoPosition(sampleAngle));
                        canpickUp = true;
                        pickupTimer.reset();
                    }
                    if(pickupTimer.seconds()>0.5 && canpickUp){
                        robotConfig.intakeWristRightServo.setPosition(ServoConstants.INTAKE_WRIST_RIGHT_DOWN_POSITION+0.03);
                        robotConfig.intakeWristLeftServo.setPosition(ServoConstants.INTAKE_WRIST_LEFT_DOWN_POSITION+0.03);
                    }
                    if(pickupTimer.seconds()>0.8 && canpickUp){
                        robotConfig.intakeClawServo.setPosition(ServoConstants.INTAKE_CLAW_CLOSED_POSITION);
                    }
                    if(pickupTimer.seconds()>1.1){
                        startTransferTimer = true;
                        hasTransfered = false;
                        follower.followPath(return2);
                        setPathState(20);
                    }
                }
                break;
            case 20:
                transfer(0.6, false);
                if(hasTransfered){
                    setPathState(25);
                }
                break;
            case 21:
                if(!follower.isBusy()) {
                    if(!canpickUp) {
                        currentPosition = robotConfig.intakeElbowLeftServo.getPosition();
                        robotConfig.intakeClawServo.setPosition(ServoConstants.INTAKE_CLAW_OPEN_POSITION);
                        robotConfig.outtakeClawServo.setPosition(ServoConstants.OUTTAKE_CLAW_OPEN_POSITION);
                        robotConfig.intakeWristLeftServo.setPosition(ServoConstants.INTAKE_WRIST_LEFT_HOVER_POSITION);
                        robotConfig.intakeWristRightServo.setPosition(ServoConstants.INTAKE_WRIST_RIGHT_HOVER_POSITION);
                        robotConfig.intakeElbowLeftServo.setPosition(currentPosition - retract);
                        robotConfig.intakeElbowRightServo.setPosition(currentPosition - retract);
                        robotConfig.intakeWristServo.setPosition(ServoConstants.INTAKE_WRIST_DOWN);
                        robotConfig.intakeWristRotServo.setPosition(mapAngleToServoPosition(sampleAngle));
                        canpickUp = true;
                        pickupTimer.reset();
                    }
                    if(pickupTimer.seconds()>0.5 && canpickUp){
                        robotConfig.intakeWristRightServo.setPosition(ServoConstants.INTAKE_WRIST_RIGHT_DOWN_POSITION);
                        robotConfig.intakeWristLeftServo.setPosition(ServoConstants.INTAKE_WRIST_LEFT_DOWN_POSITION);
                    }
                    if(pickupTimer.seconds()>0.8 && canpickUp){
                        robotConfig.intakeClawServo.setPosition(ServoConstants.INTAKE_CLAW_CLOSED_POSITION);
                    }
                    if(pickupTimer.seconds()>1.1){
                        startTransferTimer = true;
                        hasTransfered = false;
                        follower.followPath(return2);
                        setPathState(22);
                    }
                }
                break;
            case 22:
                transfer(0.6, false);
                if(hasTransfered){
                    setPathState(25);
                }
                break;
            case 23:
                if(!follower.isBusy()) {
                    if(!canpickUp) {
                        currentPosition = robotConfig.intakeElbowLeftServo.getPosition();
                        List<double[]> newSamples = processor.getRealPositions();
                        List<Double> newAngles = processor.getSampleAngles();

                        for (int i = 0; i < newSamples.size(); i++) {
                            double newY = newSamples.get(i)[1];
                            // Check if this sample is the same as the previously detected one
                            if (newY<=1 && newY>=-1) {
                                sampleAngle = newAngles.get(i);

                            }
                        }
                        robotConfig.outtakeClawServo.setPosition(ServoConstants.OUTTAKE_CLAW_OPEN_POSITION);
                        robotConfig.intakeWristLeftServo.setPosition(ServoConstants.INTAKE_WRIST_LEFT_HOVER_POSITION);
                        robotConfig.intakeWristRightServo.setPosition(ServoConstants.INTAKE_WRIST_RIGHT_HOVER_POSITION);
                        robotConfig.intakeElbowLeftServo.setPosition(currentPosition - retract);
                        robotConfig.intakeElbowRightServo.setPosition(currentPosition - retract);
                        robotConfig.intakeWristServo.setPosition(ServoConstants.INTAKE_WRIST_DOWN);
                        robotConfig.intakeWristRotServo.setPosition(mapAngleToServoPosition(sampleAngle));
                        canpickUp = true;
                        pickupTimer.reset();
                    }
                    if(pickupTimer.seconds()>0.5 && canpickUp){
                        robotConfig.intakeWristRightServo.setPosition(ServoConstants.INTAKE_WRIST_RIGHT_DOWN_POSITION+0.03);
                        robotConfig.intakeWristLeftServo.setPosition(ServoConstants.INTAKE_WRIST_LEFT_DOWN_POSITION+0.03);
                    }
                    if(pickupTimer.seconds()>0.8 && canpickUp){
                        robotConfig.intakeClawServo.setPosition(ServoConstants.INTAKE_CLAW_CLOSED_POSITION);
                    }
                    if(pickupTimer.seconds()>1.1){
                        startTransferTimer = true;
                        hasTransfered = false;
                        follower.followPath(return2);
                        setPathState(24);
                    }
                }
                break;
            case 24:
                transfer(0.6, false);
                if(hasTransfered){
                    setPathState(25);
                }
                break;
            case 25:
                if(!follower.isBusy()){
                    scoreSample(0.4);
                }
                if(hasScored) {
                    follower.followPath(subPath2);
                    cameraTimer.reset();
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
        follower.setStartingPose(sub1);
        initializeCamera();
        resetServosToInit();
        buildPaths();

    }

    /**
     * This is the main loop of the OpMode, it will run repeatedly after clicking "Play".
     **/
    @Override
    public void loop() {

        follower.update();
        try {
            autonomousPathUpdate();
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }
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
        if(gamepad1.a)
            sub1X+=1;
        telemetry.addData("sub1x", sub1X);
        telemetry.update();
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
    public void liftSlides(){
        targetPosition = OuttakeConstants.OUTTAKE_MAX_POSITION;
        robotConfig.outtakeElbowLeftServo.setPosition(ServoConstants.OUTTAKE_ELBOW_LEFT_PLACE_SAMPLE_VERTICAL);
        robotConfig.outtakeElbowRightServo.setPosition(ServoConstants.OUTTAKE_ELBOW_RIGHT_PLACE_SAMPLE_VERTICAL);
    }
    public double mapAngleToServoPosition(double angle) {
        double newAngle =(angle - Math.PI / 2) * 180 / Math.PI;
        double servoMin = 0.39, servoMax = 0.98;
        //newAngle = Math.max(0, Math.min(180, newAngle));
        return 0.00327 * newAngle + 0.41;
    }
    private void extendArmGradually() throws InterruptedException {
        // Gradually extend the arm until a sample is detected
        if (currentPosition < MAX_POSITION) {
            currentPosition += EXTEND_STEP_SIZE2;
            currentPosition = Math.min(currentPosition, MAX_POSITION);
            robotConfig.intakeElbowLeftServo.setPosition(currentPosition);
            robotConfig.intakeElbowRightServo.setPosition(currentPosition);
            //sleep(10); // Delay to smooth the extension
        }
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


    private void initializeCamera() {
        processor = new SampleOrientationProcessor(telemetry);
        visionPortal = VisionPortal.easyCreateWithDefaults(
                hardwareMap.get(WebcamName.class, "camera"), // Assuming "camera" is the webcam name
                processor
        );
    }



    private void scoreSample(double timer){
        if(startTimer){
            startHoverTimer = true;
            retractTimer.reset();
            hasFinishedPickUp = false;
            startTimer = false;
        }
        targetPosition = OuttakeConstants.OUTTAKE_MAX_POSITION;
        if (retractTimer.seconds()<timer) {
            robotConfig.outtakeElbowRightServo.setPosition(ServoConstants.OUTTAKE_ELBOW_RIGHT_PLACE_SAMPLE_POSITION);
            robotConfig.outtakeElbowLeftServo.setPosition(ServoConstants.OUTTAKE_ELBOW_LEFT_PLACE_SAMPLE_POSITION);
            robotConfig.outtakeWristYServo.setPosition(ServoConstants.OUTTAKE_WRIST_Y_PLACE_POSITION);
        }
        if (retractTimer.seconds() > timer) { // 1.2
            robotConfig.outtakeClawServo.setPosition(ServoConstants.OUTTAKE_CLAW_OPEN_POSITION);
            if (robotConfig.outtakeClawServo.getPosition() == ServoConstants.OUTTAKE_CLAW_OPEN_POSITION) {
                if (retractTimer.seconds() > timer + 0.1) { // 1.5
                    robotConfig.outtakeElbowRightServo.setPosition(ServoConstants.OUTTAKE_ELBOW_RIGHT_PICKUP_POSITION);
                    robotConfig.outtakeElbowLeftServo.setPosition(ServoConstants.OUTTAKE_ELBOW_LEFT_PICKUP_POSITION);
                    robotConfig.outtakeWristYServo.setPosition(ServoConstants.OUTTAKE_WRIST_Y_TRANSFER_POSITION);
                    hasScored = true;
                }

            }
        }
    }
    private void setSliderMinPosition(){
        targetPosition = OuttakeConstants.OUTTAKE_MIN_POSITION;
    }
    private void transfer(double timer, boolean liftSlides) {
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
        if(hasTransfered && liftSlides) {
            targetPosition = OuttakeConstants.OUTTAKE_MAX_POSITION;
            robotConfig.outtakeElbowLeftServo.setPosition(ServoConstants.OUTTAKE_ELBOW_LEFT_PLACE_SAMPLE_VERTICAL);
            robotConfig.outtakeElbowRightServo.setPosition(ServoConstants.OUTTAKE_ELBOW_RIGHT_PLACE_SAMPLE_VERTICAL);
        }
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