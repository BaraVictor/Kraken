package org.firstinspires.ftc.teamcode.competitional.auto;

import static java.lang.Math.toRadians;

import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierCurve;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;
import com.pedropathing.util.Constants;
import com.pedropathing.util.CustomPIDFCoefficients;
import com.pedropathing.util.PIDFController;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.configurations.RobotConfig;
import org.firstinspires.ftc.teamcode.constants.OuttakeConstants;
import org.firstinspires.ftc.teamcode.constants.ServoConstants;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.FConstants;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.LConstants;

/**
 * This is an example auto that showcases movement and control of two servos autonomously.
 * It is a 0+4 (Specimen + Sample) bucket auto. It scores a neutral preload and then pickups 3 samples from the ground and scores them before parking.
 * There are examples of different ways to build paths.
 * A path progression method has been created and can advance based on time, position, or other factors.
 *
 * @author Baron Henderson - 20077 The Indubitables
 * @version 2.0, 11/28/2024
 */

@Autonomous(name = "Spec Auto ", group = "A. Competitional")
public class SpecAuto extends OpMode {

    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer; //  ??

    /** This is the variable where we store the state of our auto.
     * It is used by the pathUpdate method. */
    private int pathState;
    private RobotConfig robotConfig;
    private PIDFController pidfController;

    public static double P = 0.007;
    public static double I = 0;
    public static double D = 0;
    public static double F = 0;
    public static double K = 0;
    public static double targetPosition = 0;
    private boolean areSlidesDown = false;

    private ElapsedTime placeSpacimen = new ElapsedTime();
    private boolean placeSpecimenOnChamber = true;
    private boolean canPickup = false;
    private boolean da = false;

    /* Create and Define Poses + Paths
     * Poses are built with three constructors: x, y, and heading (in Radians).
     * Pedro uses 0 - 144 for x and y, with 0, 0 being on the bottom left.
     * (For Into the Deep, this would be Blue Observation Zone (0,0) to Red Observation Zone (144,144).)
     * Even though Pedro uses a different coordinate system than RR, you can convert any roadrunner pose by adding +72 both the x and y.
     * This visualizer is very easy to use to find and create paths/pathchains/poses: <https://pedro-path-generator.vercel.app/>
     * Lets assume our robot is 18 by 18 inches
     * Lets assume the Robot is facing the human player and we want to score in the bucket */

    /** Start Pose of our robot */
        private final Pose startPose = new Pose(10, 67, toRadians(0));

        private  final Pose scorePreload = new Pose(45,67, toRadians(0));

    private final Pose sample1 = new Pose(62, 22, toRadians(0));

    private final Pose sample1Control1= new Pose(0, 25, toRadians(0));

    private final Pose sample1Control2 = new Pose(65, 40, toRadians(0));

    private final Pose push1 = new Pose(30, 22, toRadians(0));


    private final Pose sample2 = new Pose(62, 12, toRadians(0));

    private final Pose sample2Control1= new Pose(60, 30, toRadians(0));

    private final Pose push2= new Pose(30, 12, toRadians(0));



    private final Pose sample3 = new Pose(62, 2, toRadians(0));

    private final Pose sample3Control1= new Pose(60, 25, toRadians(0));

    
    private final Pose grab1= new Pose(12.8,2, toRadians(0));


    private final Pose grabLineup= new Pose(25, 27, toRadians(0));

    private final Pose grabPickup= new Pose(14, 27, toRadians(0));



    private final Pose placeControl = new Pose(20, 70, toRadians(0));

    private final Pose place1 = new Pose(45.4   , 73, toRadians(0));

    private final Pose place2 = new Pose(45.5, 70, toRadians(0));

    private final Pose place3 = new Pose(45.7, 67, toRadians(0));

    private final Pose place4 = new Pose(46.2, 64, toRadians(0));

    private final Pose parkPose = new Pose(7, 27, toRadians(-90));

    private PathChain park, preloadDrop, samplePush1, firstPlace, secondGrab, secondPlace, thirdGrab, thirdPlace, fourthGrab, fourthPlace; //?
    
    public void buildPaths() {
        preloadDrop = follower.pathBuilder()
                .addPath(new BezierLine(new Point(startPose), new Point(scorePreload)))
                .setConstantHeadingInterpolation(scorePreload.getHeading())
                .addParametricCallback(0.1, () -> lineupSpec())
                .addParametricCallback(0.8,()->follower.setMaxPower(0.5))
                .setZeroPowerAccelerationMultiplier(4.0)
                .setPathEndTimeoutConstraint(0)
                .build();

        samplePush1 = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(scorePreload), new Point(sample1Control1), new Point(sample1Control2), new Point(sample1)))
                .setConstantHeadingInterpolation(scorePreload.getHeading())
                .setPathEndTimeoutConstraint(0)
                .addParametricCallback(0, ()->follower.setMaxPower(0.9))
                .addParametricCallback(0.1, () -> pickupSpec())

                .addPath(new BezierLine(new Point(sample1), new Point(push1)))
                .setConstantHeadingInterpolation(scorePreload.getHeading())
                .setZeroPowerAccelerationMultiplier(5.0)
                .setPathEndTimeoutConstraint(0)

                .addPath(new BezierCurve(new Point(push1), new Point(sample2Control1),new Point(sample2)))
                .setConstantHeadingInterpolation(scorePreload.getHeading())
                .addParametricCallback(0.8, ()->follower.setMaxPower(0.8))
                .setZeroPowerAccelerationMultiplier(3.0)
                .setPathEndTimeoutConstraint(0)

                .addPath(new BezierLine(new Point(sample2), new Point(push2)))
                .setConstantHeadingInterpolation(scorePreload.getHeading())
                .addParametricCallback(0, ()->follower.setMaxPower(0.9))
                .setZeroPowerAccelerationMultiplier(5.0)
                .setPathEndTimeoutConstraint(0)

                .addPath(new BezierCurve(new Point(push2), new Point(sample3Control1),new Point(sample3)))
                .setConstantHeadingInterpolation(scorePreload.getHeading())
                .addParametricCallback(0.7, () ->follower.setMaxPower(0.6))
                .setZeroPowerAccelerationMultiplier(3.0)
                .setPathEndTimeoutConstraint(0)

                .addPath(new BezierLine(new Point(sample3), new Point(grab1)))
                .setConstantHeadingInterpolation(scorePreload.getHeading())
                .addParametricCallback(0, ()->follower.setMaxPower(0.9))
                .addParametricCallback(0.7, () -> follower.setMaxPower(0.6))
                .setZeroPowerAccelerationMultiplier(3.5)
                .setPathEndTimeoutConstraint(0)
                .build();

        firstPlace = follower.pathBuilder()
                .addPath(new BezierLine(new Point(grab1), new Point(place1)))
                .setConstantHeadingInterpolation(grabLineup.getHeading())
                .addParametricCallback(0, ()-> follower.setMaxPower(0.9))
                .addParametricCallback(0.8, ()->follower.setMaxPower(0.5))
                .setZeroPowerAccelerationMultiplier(4.0)
                .setPathEndTimeoutConstraint(0)
                .build();

        secondGrab = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(place1), new Point(placeControl), new Point(grabLineup)))
                .setConstantHeadingInterpolation(grabLineup.getHeading())
                .addParametricCallback(0.1, () -> pickupSpec())
                .addParametricCallback(0, ()-> follower.setMaxPower(0.8))
                .addParametricCallback(0.8, ()-> follower.setMaxPower(0.4))
                .setZeroPowerAccelerationMultiplier(1.0)
                .setPathEndTimeoutConstraint(0)

                .addPath(new BezierLine(new Point(grabLineup), new Point(grabPickup)))
                .setConstantHeadingInterpolation(grabLineup.getHeading())
                .setZeroPowerAccelerationMultiplier(3.0)
                .setPathEndTimeoutConstraint(0)
                .addParametricCallback(0, ()->follower.setMaxPower(0.5))
                .build();
        secondPlace = follower.pathBuilder()
                .addPath(new BezierLine(new Point(grabPickup), new Point(place2)))
                .setLinearHeadingInterpolation(grab1.getHeading(), grabLineup.getHeading())
                .setZeroPowerAccelerationMultiplier(4.0)
                .setPathEndTimeoutConstraint(0)
                .addParametricCallback(0, ()-> follower.setMaxPower(0.9))
                .addParametricCallback(0.8, ()-> follower.setMaxPower(0.5))

                .build();

        thirdGrab = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(place2),new Point(placeControl), new Point(grabLineup)))
                .setLinearHeadingInterpolation(place1.getHeading(), grabLineup.getHeading())
                .addParametricCallback(0.1, () -> pickupSpec())
                .addParametricCallback(0, ()-> follower.setMaxPower(0.8))
                .addParametricCallback(0.8, ()-> follower.setMaxPower(0.4))
                .setZeroPowerAccelerationMultiplier(1.0)
                .setPathEndTimeoutConstraint(0)

                .addPath(new BezierLine(new Point(grabLineup), new Point(grabPickup)))
                .setConstantHeadingInterpolation(grabLineup.getHeading())
                .setZeroPowerAccelerationMultiplier(3.0)
                .setPathEndTimeoutConstraint(0)
                .addParametricCallback(0, ()->follower.setMaxPower(0.5))
                .build();
        thirdPlace = follower.pathBuilder()

                .addPath(new BezierLine(new Point(grabPickup), new Point(place3)))
                .setLinearHeadingInterpolation(grab1.getHeading(), grabLineup.getHeading())
                .setZeroPowerAccelerationMultiplier(4.0)
                .addParametricCallback(0, ()-> follower.setMaxPower(0.9))
                .addParametricCallback(0.8, ()-> follower.setMaxPower(0.5))
                .setPathEndTimeoutConstraint(0)
                .build();

        fourthGrab = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(place3), new Point(placeControl),new Point(grabLineup)))
                .setLinearHeadingInterpolation(place1.getHeading(), grabLineup.getHeading())
                .addParametricCallback(0.1, () -> pickupSpec())
                .addParametricCallback(0, ()-> follower.setMaxPower(0.8))
                .addParametricCallback(0.8, ()-> follower.setMaxPower(0.4))
                .setZeroPowerAccelerationMultiplier(1.0)
                .setPathEndTimeoutConstraint(0)

                .addPath(new BezierLine(new Point(grabLineup), new Point(grabPickup)))
                .setConstantHeadingInterpolation(grabLineup.getHeading())
                .setZeroPowerAccelerationMultiplier(3.0)
                .setPathEndTimeoutConstraint(0)
                .addParametricCallback(0, ()->follower.setMaxPower(0.5))
                .build();
        fourthPlace = follower.pathBuilder()
                .addPath(new BezierLine(new Point(grabPickup), new Point(place4)))
                .setLinearHeadingInterpolation(grab1.getHeading(), grabLineup.getHeading())
                .setZeroPowerAccelerationMultiplier(4.0)
                .setPathEndTimeoutConstraint(0)
                .addParametricCallback(0, ()-> follower.setMaxPower(0.9))
                .addParametricCallback(0.8, ()-> follower.setMaxPower(0.5))
                .build();

        park = follower.pathBuilder()
                .addPath((new BezierCurve(new Point(place4), new Point(placeControl),new Point(parkPose))))
                .setLinearHeadingInterpolation(grab1.getHeading(), parkPose.getHeading())
                .addParametricCallback(0, ()-> follower.setMaxPower(1))
                .addParametricCallback(0.3, () -> intakeTeleOp())
                .addParametricCallback(0.3, () -> outtakeTeleOp())
                .build();

    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                follower.followPath(preloadDrop,true);
                placeSpecimenOnChamber = true;
                setPathState(1);
                break;
            case 1:
                if(!follower.isBusy()){
                        if(placeSpecimenOnChamber){
                            targetPosition = OuttakeConstants.OUTTAKE_SECOND_SPECIMEN_BAR_SCORE;
                            robotConfig.setOuttakeServoPositions(
                                ServoConstants.OUTTAKE_CLAW_CLOSED_POSITION,
                                ServoConstants.OUTTAKE_WRIST_ROT_0_DEGREES,
                                ServoConstants.OUTTAKE_WRIST_Y_PLACE_SPECIMEN_POSITION,
                                ServoConstants.OUTTAKE_ELBOW_RIGHT_PLACE_SPECIMEN_POSITION + 0.1,
                                ServoConstants.OUTTAKE_ELBOW_LEFT_PLACE_SPECIMEN_POSITION + 0.1
                            );
                            placeSpecimenOnChamber = false;
                            placeSpacimen.reset();
                        }
                    //liftSlides();
                }
                if(!placeSpecimenOnChamber){
                    if(placeSpacimen.milliseconds() > 150) {
                        robotConfig.setOuttakeServoPositions(
                                ServoConstants.OUTTAKE_CLAW_OPEN_POSITION,
                                ServoConstants.OUTTAKE_WRIST_ROT_0_DEGREES,
                                ServoConstants.OUTTAKE_WRIST_Y_PLACE_SPECIMEN_POSITION,
                                ServoConstants.OUTTAKE_ELBOW_RIGHT_PLACE_SPECIMEN_POSITION,
                                ServoConstants.OUTTAKE_ELBOW_LEFT_PLACE_SPECIMEN_POSITION
                        );
                        targetPosition = OuttakeConstants.OUTTAKE_SECOND_SPECIMEN_BAR_RELEASE; //220
                        follower.followPath(samplePush1);
                        setPathState(2);
                    }
                }
                break;

            case 2:
                if(follower.getCurrentTValue()==1 && !canPickup) {
                    da = true;
                    canPickup = true;
                    placeTimer.reset();
                }

                if(canPickup)
                {
                    if(placeTimer.milliseconds()>40)
                        robotConfig.outtakeClawServo.setPosition(ServoConstants.OUTTAKE_CLAW_CLOSED_POSITION);
                    if(placeTimer.milliseconds()>80)
                        lineupSpec();
                    if(placeTimer.milliseconds()>100) {
                        follower.followPath(firstPlace);
                        placeSpecimenOnChamber = true;
                        setPathState(3);
                    }
                }
                break;
            case 3:
                if(!follower.isBusy()){
                    if(placeSpecimenOnChamber){
                        targetPosition = OuttakeConstants.OUTTAKE_SECOND_SPECIMEN_BAR_SCORE;
                        robotConfig.setOuttakeServoPositions(
                                ServoConstants.OUTTAKE_CLAW_CLOSED_POSITION,
                                ServoConstants.OUTTAKE_WRIST_ROT_0_DEGREES,
                                ServoConstants.OUTTAKE_WRIST_Y_PLACE_SPECIMEN_POSITION,
                                ServoConstants.OUTTAKE_ELBOW_RIGHT_PLACE_SPECIMEN_POSITION + 0.1,
                                ServoConstants.OUTTAKE_ELBOW_LEFT_PLACE_SPECIMEN_POSITION + 0.1
                        );
                        placeSpecimenOnChamber = false;
                        placeSpacimen.reset();
                    }
                    //liftSlides();
                }
                if(!placeSpecimenOnChamber){
                    if(placeSpacimen.milliseconds() > 150) {
                        robotConfig.setOuttakeServoPositions(
                                ServoConstants.OUTTAKE_CLAW_OPEN_POSITION,
                                ServoConstants.OUTTAKE_WRIST_ROT_0_DEGREES,
                                ServoConstants.OUTTAKE_WRIST_Y_PLACE_SPECIMEN_POSITION,
                                ServoConstants.OUTTAKE_ELBOW_RIGHT_PLACE_SPECIMEN_POSITION,
                                ServoConstants.OUTTAKE_ELBOW_LEFT_PLACE_SPECIMEN_POSITION
                        );
                        targetPosition = OuttakeConstants.OUTTAKE_SECOND_SPECIMEN_BAR_RELEASE; //220
                        follower.followPath(secondGrab);

                        canPickup= false;
                        setPathState(4);
                    }
                }
                break;

            case 4:
                if(follower.getCurrentTValue()==1 && !canPickup) {
                    da = true;
                    canPickup = true;
                    placeTimer.reset();
                }

                if(canPickup)
                {
                    if(placeTimer.milliseconds()>50)
                        robotConfig.outtakeClawServo.setPosition(ServoConstants.OUTTAKE_CLAW_CLOSED_POSITION);
                    if(placeTimer.milliseconds()>80)
                        lineupSpec();
                    if(placeTimer.milliseconds()>100) {
                        follower.followPath(secondPlace);
                        placeSpecimenOnChamber = true;
                        setPathState(5);
                    }
                }
                break;
            case 5:
                if(!follower.isBusy()){
                    if(placeSpecimenOnChamber){
                        targetPosition = OuttakeConstants.OUTTAKE_SECOND_SPECIMEN_BAR_SCORE;
                        robotConfig.setOuttakeServoPositions(
                                ServoConstants.OUTTAKE_CLAW_CLOSED_POSITION,
                                ServoConstants.OUTTAKE_WRIST_ROT_0_DEGREES,
                                ServoConstants.OUTTAKE_WRIST_Y_PLACE_SPECIMEN_POSITION,
                                ServoConstants.OUTTAKE_ELBOW_RIGHT_PLACE_SPECIMEN_POSITION + 0.1,
                                ServoConstants.OUTTAKE_ELBOW_LEFT_PLACE_SPECIMEN_POSITION + 0.1
                        );
                        placeSpecimenOnChamber = false;
                        placeSpacimen.reset();
                    }
                    //liftSlides();
                }
                if(!placeSpecimenOnChamber){
                    if(placeSpacimen.milliseconds() > 150) {
                        robotConfig.setOuttakeServoPositions(
                                ServoConstants.OUTTAKE_CLAW_OPEN_POSITION,
                                ServoConstants.OUTTAKE_WRIST_ROT_0_DEGREES,
                                ServoConstants.OUTTAKE_WRIST_Y_PLACE_SPECIMEN_POSITION,
                                ServoConstants.OUTTAKE_ELBOW_RIGHT_PLACE_SPECIMEN_POSITION,
                                ServoConstants.OUTTAKE_ELBOW_LEFT_PLACE_SPECIMEN_POSITION
                        );
                        targetPosition = OuttakeConstants.OUTTAKE_SECOND_SPECIMEN_BAR_RELEASE; //220
                        follower.followPath(thirdGrab);

                        canPickup= false;
                        setPathState(6);
                    }
                }
                break;

            case 6:
                if(follower.getCurrentTValue()==1 && !canPickup) {
                    da = true;
                    canPickup = true;
                    placeTimer.reset();
                }

                if(canPickup)
                {
                    if(placeTimer.milliseconds()>50)
                        robotConfig.outtakeClawServo.setPosition(ServoConstants.OUTTAKE_CLAW_CLOSED_POSITION);
                    if(placeTimer.milliseconds()>80)
                        lineupSpec();
                    if(placeTimer.milliseconds()>100) {
                        follower.followPath(thirdPlace);
                        placeSpecimenOnChamber = true;
                        setPathState(7);
                    }
                }
                break;
            case 7:
                if(!follower.isBusy()){
                    if(placeSpecimenOnChamber){
                        targetPosition = OuttakeConstants.OUTTAKE_SECOND_SPECIMEN_BAR_SCORE;
                        robotConfig.setOuttakeServoPositions(
                                ServoConstants.OUTTAKE_CLAW_CLOSED_POSITION,
                                ServoConstants.OUTTAKE_WRIST_ROT_0_DEGREES,
                                ServoConstants.OUTTAKE_WRIST_Y_PLACE_SPECIMEN_POSITION,
                                ServoConstants.OUTTAKE_ELBOW_RIGHT_PLACE_SPECIMEN_POSITION + 0.1,
                                ServoConstants.OUTTAKE_ELBOW_LEFT_PLACE_SPECIMEN_POSITION + 0.1
                        );
                        placeSpecimenOnChamber = false;
                        placeSpacimen.reset();
                    }
                    //liftSlides();
                }
                if(!placeSpecimenOnChamber){
                    if(placeSpacimen.milliseconds() > 150) {
                        robotConfig.setOuttakeServoPositions(
                                ServoConstants.OUTTAKE_CLAW_OPEN_POSITION,
                                ServoConstants.OUTTAKE_WRIST_ROT_0_DEGREES,
                                ServoConstants.OUTTAKE_WRIST_Y_PLACE_SPECIMEN_POSITION,
                                ServoConstants.OUTTAKE_ELBOW_RIGHT_PLACE_SPECIMEN_POSITION,
                                ServoConstants.OUTTAKE_ELBOW_LEFT_PLACE_SPECIMEN_POSITION
                        );
                        targetPosition = OuttakeConstants.OUTTAKE_SECOND_SPECIMEN_BAR_RELEASE; //220
                        follower.followPath(fourthGrab);

                        canPickup= false;
                        setPathState(8);
                    }
                }
                break;
            case 8:
                if(follower.getCurrentTValue()==1 && !canPickup) {
                    da = true;
                    canPickup = true;
                    placeTimer.reset();
                }

                if(canPickup)
                {
                    if(placeTimer.milliseconds()>50)
                        robotConfig.outtakeClawServo.setPosition(ServoConstants.OUTTAKE_CLAW_CLOSED_POSITION);
                    if(placeTimer.milliseconds()>80)
                        lineupSpec();
                    if(placeTimer.milliseconds()>100) {
                        follower.followPath(fourthPlace);
                        placeSpecimenOnChamber = true;
                        setPathState(9);
                    }
                }
                break;
            case 9:
                if(!follower.isBusy()){
                    if(placeSpecimenOnChamber){
                        targetPosition = OuttakeConstants.OUTTAKE_SECOND_SPECIMEN_BAR_SCORE;
                        robotConfig.setOuttakeServoPositions(
                                ServoConstants.OUTTAKE_CLAW_CLOSED_POSITION,
                                ServoConstants.OUTTAKE_WRIST_ROT_0_DEGREES,
                                ServoConstants.OUTTAKE_WRIST_Y_PLACE_SPECIMEN_POSITION,
                                ServoConstants.OUTTAKE_ELBOW_RIGHT_PLACE_SPECIMEN_POSITION + 0.1,
                                ServoConstants.OUTTAKE_ELBOW_LEFT_PLACE_SPECIMEN_POSITION + 0.1
                        );
                        placeSpecimenOnChamber = false;
                        placeSpacimen.reset();
                    }
                    //liftSlides();
                }
                if(!placeSpecimenOnChamber){
                    if(placeSpacimen.milliseconds() > 150) {
                        robotConfig.setOuttakeServoPositions(
                                ServoConstants.OUTTAKE_CLAW_OPEN_POSITION,
                                ServoConstants.OUTTAKE_WRIST_ROT_0_DEGREES,
                                ServoConstants.OUTTAKE_WRIST_Y_PLACE_SPECIMEN_POSITION,
                                ServoConstants.OUTTAKE_ELBOW_RIGHT_PLACE_SPECIMEN_POSITION,
                                ServoConstants.OUTTAKE_ELBOW_LEFT_PLACE_SPECIMEN_POSITION
                        );
                        targetPosition = OuttakeConstants.OUTTAKE_SECOND_SPECIMEN_BAR_RELEASE; //220
                        follower.followPath(park);

                        canPickup= false;
                        setPathState(-1);
                    }
                }
                break;
            case 10:
                if(!follower.isBusy()){
                    follower.followPath(park);
                    setPathState(-1);
                }
                break;

        }
    }

    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }

    @Override
    public void loop() {

        follower.update();
        autonomousPathUpdate();
        controlPID();
        // Feedback to Driver Hub
        telemetry.addData("path state", pathState);
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.addData("isBusy", follower.isBusy());
        telemetry.addData("TValue", follower.getCurrentTValue());
        telemetry.addData("da", da);
        telemetry.update();
    }

    @Override
    public void init() {
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
        resetServosToInit();
        buildPaths();

    }

    @Override
    public void init_loop() {}

    @Override
    public void start() {
        opmodeTimer.resetTimer();
        setPathState(0);
    }

    @Override
    public void stop() {
    }

    public void placeSpecimen(){

    }//?

    private void updatePIDFController () {
        CustomPIDFCoefficients coefficients = new CustomPIDFCoefficients(P, I, D, F);
        pidfController = new PIDFController(coefficients);
    }

    private void resetServosToInit () {
        robotConfig.setOuttakeServoPositions(
                ServoConstants.OUTTAKE_CLAW_CLOSED_POSITION,
                ServoConstants.OUTTAKE_WRIST_ROT_0_DEGREES,
                ServoConstants.OUTTAKE_WRIST_Y_INIT,
                ServoConstants.OUTTAKE_ELBOW_RIGHT_INIT,
                ServoConstants.OUTTAKE_ELBOW_LEFT_INIT
        );
        robotConfig.setIntakeServoPositions(
                ServoConstants.INTAKE_ELBOW_RIGHT_RETRACTED_POSITION,
                ServoConstants.INTAKE_ELBOW_LEFT_RETRACTED_POSITION,
                ServoConstants.INTAKE_WRIST_INIT,
                ServoConstants.INTAKE_WRIST_RIGHT_INIT,
                ServoConstants.INTAKE_WRIST_LEFT_INIT,
                ServoConstants.INTAKE_CLAW_OPEN_POSITION,
                ServoConstants.INTAKE_WRIST_ROT_0_DEGREES
        );
    }
    private void controlPID(){
        pidfController.setTargetPosition(targetPosition);
        pidfController.updatePosition(robotConfig.upMotor.getCurrentPosition());

        double powerUp = pidfController.runPIDF() + K;

        if (Math.abs(robotConfig.upMotor.getCurrentPosition() - targetPosition) <= OuttakeConstants.TOLERANCE) {
            powerUp = 0;
        }
        if(robotConfig.upMotor.getCurrentPosition()>10)
            areSlidesDown = false;
        if(robotConfig.upMotor.getCurrentPosition()<100 && robotConfig.upMotor.getVelocity()<0.05 && targetPosition== OuttakeConstants.OUTTAKE_MIN_POSITION){
            RobotConfig.upMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
            RobotConfig.midMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
            RobotConfig.downMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

            RobotConfig.upMotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
            RobotConfig.midMotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
            RobotConfig.downMotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
            areSlidesDown = true;
        }
        if(targetPosition == OuttakeConstants.OUTTAKE_MIN_POSITION && areSlidesDown){
            robotConfig.upMotor.setPower(0);
            robotConfig.midMotor.setPower(0);
            robotConfig.downMotor.setPower(0);
        }
        if(targetPosition == OuttakeConstants.OUTTAKE_MIN_POSITION && robotConfig.upMotor.getCurrentPosition()>10){
            robotConfig.upMotor.setPower(-0.9);
            robotConfig.midMotor.setPower(-0.9);
            robotConfig.downMotor.setPower(-0.9);

        }
        else {
            robotConfig.upMotor.setPower(powerUp);
            robotConfig.midMotor.setPower(powerUp);
            robotConfig.downMotor.setPower(powerUp);
        }
        robotConfig.upMotor.setPower(powerUp);
        robotConfig.midMotor.setPower(powerUp);
        robotConfig.downMotor.setPower(powerUp);
    }

    public void lowSpeed() {
        follower.setMaxPower(0.3);
    }

        public void highSpeed() {
        follower.setMaxPower(0.9);
        }


    private void lineupSpec(){
        robotConfig.setOuttakeServoPositions(
                ServoConstants.OUTTAKE_CLAW_CLOSED_POSITION,
                ServoConstants.OUTTAKE_WRIST_ROT_0_DEGREES,
                ServoConstants.OUTTAKE_WRIST_Y_PLACE_SPECIMEN_POSITION,
                ServoConstants.OUTTAKE_ELBOW_RIGHT_PLACE_SPECIMEN_POSITION,
                ServoConstants.OUTTAKE_ELBOW_LEFT_PLACE_SPECIMEN_POSITION
        );
        targetPosition = OuttakeConstants.OUTTAKE_SECOND_SPECIMEN_BAR;
    }

    private ElapsedTime placeTimer = new ElapsedTime();

    private void pickupSpec(){
        targetPosition = OuttakeConstants.OUTTAKE_MIN_POSITION;
        robotConfig.setOuttakeServoPositions(
                ServoConstants.OUTTAKE_CLAW_OPEN_POSITION,
                ServoConstants.OUTTAKE_WRIST_ROT_180_DEGREES,
                ServoConstants.OUTTAKE_WRIST_Y_PICKUP_SPECIMEN_POSITION,
                ServoConstants.OUTTAKE_ELBOW_RIGHT_SPECIMEN_POSITION,
                ServoConstants.OUTTAKE_ELBOW_LEFT_SPECIMEN_POSITION
        );
    }

    private void outtakeTeleOp() {
        targetPosition = OuttakeConstants.OUTTAKE_MIN_POSITION;
        robotConfig.setOuttakeServoPositions(
                ServoConstants.OUTTAKE_CLAW_CLOSED_POSITION,
                ServoConstants.OUTTAKE_WRIST_ROT_180_DEGREES,
                ServoConstants.OUTTAKE_WRIST_Y_TRANSFER_POSITION,
                ServoConstants.OUTTAKE_ELBOW_RIGHT_PICKUP_POSITION,
                ServoConstants.OUTTAKE_ELBOW_LEFT_PICKUP_POSITION
        );
    }

    private void intakeTeleOp(){
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

}
