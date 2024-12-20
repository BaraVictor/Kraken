package org.firstinspires.ftc.teamcode.noncompetitional.auto;


import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import  com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.configurations.RobotConfig;
import org.firstinspires.ftc.teamcode.pedroPathing.follower.*;
import org.firstinspires.ftc.teamcode.pedroPathing.localization.Pose;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierLine;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Path;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.PathChain;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Point;
import org.firstinspires.ftc.teamcode.pedroPathing.util.Timer;

import org.firstinspires.ftc.teamcode.constants.OuttakeConstants;
import org.firstinspires.ftc.teamcode.constants.ServoConstants;
import org.firstinspires.ftc.teamcode.pedroPathing.util.CustomPIDFCoefficients;
import org.firstinspires.ftc.teamcode.pedroPathing.util.PIDFController;

@Autonomous(name = "SampleAuto", group = "Test")
public class Example extends OpMode {

    private RobotConfig robotConfig;
    private FtcDashboard dashboard;

    private PIDFController pidfControllerUp;
    private PIDFController pidfControllerDown;

    public static double P = 0.01;

    public static double I = 0.004;
    public static double D = 0;
    public static double F = 0.001;
    public static double K =0;
    public static double targetPosition = 0;

    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;
    private int pathState;
    private final Pose bottomLeft = new Pose(18, 109, Math.toRadians(0));
    private final Pose topLeft = new Pose(140, 108, Math.toRadians(0));
    private final Pose topRight = new Pose(140, 14, Math.toRadians(0));
    private final Pose bottomeRight = new Pose(24, 10, Math.toRadians(0));
    private final Pose pickup3Pose = new Pose(45, 120, Math.toRadians(90));
    private final Pose parkPose = new Pose(80, 100, Math.toRadians(315));
    private Path  path1;
    private PathChain path2, path3, path4;

    public void buildPaths() {

        /* There are two major types of paths components: BezierCurves and BezierLines.
         *    * BezierCurves are curved, and require >= 3 points. There are the start and end points, and the control points.
         *    - Control points manipulate the curve between the start and end points.
         *    - A good visualizer for this is [this](https://pedro-path-generator.vercel.app/).
         *    * BezierLines are straight, and require 2 points. There are the start and end points.
         * Paths have can have heading interpolation: Constant, Linear, or Tangential
         *    * Linear heading interpolation:
         *    - Pedro will slowly change the heading of the robot from the startHeading to the endHeading over the course of the entire path.
         *    * Constant Heading Interpolation:
         *    - Pedro will maintain one heading throughout the entire path.
         *    * Tangential Heading Interpolation:
         *    - Pedro will follows the angle of the path such that the robot is always driving forward when it follows the path.
         * PathChains hold Path(s) within it and are able to hold their end point, meaning that they will holdPoint until another path is followed.
         * Here is a explanation of the difference between Paths and PathChains <https://pedropathing.com/commonissues/pathtopathchain.html> */

        /* This is our scorePreload path. We are using a BezierLine, which is a straight line. */
        path1 = new Path(new BezierLine(new Point(bottomLeft), new Point(topLeft)));
//        path1 = follower.pathBuilder()
//                .addPath(new BezierLine(new Point(bottomLeft), new Point(topLeft)))
//                .setConstantHeadingInterpolation(bottomLeft.getHeading())
//                .build();

        /* Here is an example for Constant Interpolation
        scorePreload.setConstantInterpolation(startPose.getHeading()); */

        /* This is our grabPickup1 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        path2 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(topLeft), new Point(topRight)))
                .setConstantHeadingInterpolation(topLeft.getHeading())
                .build();

        /* This is our scorePickup1 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        path3 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(topRight), new Point(bottomeRight)))
                .setConstantHeadingInterpolation(topRight.getHeading())
                .build();

        /* This is our grabPickup2 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        path4 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(bottomeRight), new Point(bottomLeft)))
                .setConstantHeadingInterpolation(bottomeRight.getHeading())
                .build();

        /* This is our scorePickup2 PathChain. We are using a single path with a BezierLine, which is a straight line. */

        /* This is our park path. We are using a BezierCurve with 3 points, which is a curved line that is curved based off of the control point */
    }

    /** This switch is called continuously and runs the pathing, at certain points, it triggers the action state.
     * Everytime the switch changes case, it will reset the timer. (This is because of the setPathState() method)
     * The followPath() function sets the follower to run the specific path, but does NOT wait for it to finish before moving on. */
    public void autonomousPathUpdate() throws InterruptedException {
        switch (pathState) {
            case 0:
                
                follower.followPath(path1);
                    setPathState(1);

                break;
            case 1:
                if(follower.getPose().getX() > (topLeft.getX() - 1) && follower.getPose().getY() > (topLeft.getY() - 1)) {
                    /* Score Preload */
                        follower.followPath(path2, true);
                        targetPosition = OuttakeConstants.OUTTAKE_MAX_POSITION;
                    robotConfig.setOuttakeServoPositions(
                            ServoConstants.OUTTAKE_CLAW_CLOSED_POSITION,
                            ServoConstants.OUTTAKE_WRIST_ROT_0_DEGREES,
                            ServoConstants.OUTTAKE_WRIST_Y_PLACE_POSITION,
                            ServoConstants.OUTTAKE_ELBOW_RIGHT_PLACE_SAMPLE_POSITION,
                            ServoConstants.OUTTAKE_ELBOW_LEFT_PLACE_SAMPLE_POSITION
                    );
                        if(robotConfig.upMotor.getCurrentPosition() > targetPosition - 10) {
                            setPathState(2);
                        }
                }
                break;
            case 2:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the pickup1Pose's position */
                if(follower.getPose().getX() > (topRight.getX() - 1) && follower.getPose().getY() > topRight.getY() - 1) {
                    /* Grab Sample */
                    /* Since this is a pathChain, we can have Pedro hold the end point while we are scoring the sample */
                    follower.followPath(path3,true);
                    targetPosition = OuttakeConstants.OUTTAKE_MIN_POSITION;
                    robotConfig.setOuttakeServoPositions(
                            ServoConstants.OUTTAKE_CLAW_CLOSED_POSITION,
                            ServoConstants.OUTTAKE_WRIST_ROT_180_DEGREES,
                            ServoConstants.OUTTAKE_WRIST_Y_TRANSFER_POSITION,
                            ServoConstants.OUTTAKE_ELBOW_RIGHT_PICKUP_POSITION,
                            ServoConstants.OUTTAKE_ELBOW_LEFT_PICKUP_POSITION
                    );
                    if(robotConfig.upMotor.getCurrentPosition() < targetPosition - 10) {
                        setPathState(3);
                    }

                }
                break;
//            case 3:
//                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
//                if(follower.getPose().getX() > (bottomeRight.getX() - 1) && follower.getPose().getY() > (bottomeRight.getY() - 1)) {
//                    /* Score Sample */
//                    /* Since this is a pathChain, we can have Pedro hold the end point while we are grabbing the sample */
//
//                    follower.followPath(path4,true);
//                        //setPathState(4);
//                }
//                break;
//            case 4:
//                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the pickup2Pose's position */
//                if(follower.getPose().getX() > (bottomLeft.getX() - 1) && follower.getPose().getY() > (bottomLeft.getY() - 1)) {
//                    /* Grab Sample */
//                    /* Since this is a pathChain, we can have Pedro hold the end point while we are scoring the sample */
//
//                    telemetry.addLine("A ajuns");
//
//                }
//                break;

        }
    }

    /** These change the states of the paths and actions
     * It will also reset the timers of the individual switches **/
    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }
    @Override
    public void init() {
        robotConfig = new RobotConfig(hardwareMap);

        updatePIDFController();

        telemetry.addLine("Outtake Sliders Initialized with PIDF and Joystick Control");
        pathTimer = new Timer();
        opmodeTimer = new Timer();

        opmodeTimer.resetTimer();

        follower = new Follower(hardwareMap);
        follower.setStartingPose(bottomLeft);

        buildPaths();

        resetServosToInit();

        dashboard = FtcDashboard.getInstance();
        dashboard.setTelemetryTransmissionInterval(25);
        telemetry = dashboard.getTelemetry();
        telemetry.addData("Status", "Initialized");
        telemetry.update();
    }
    /** This is the main loop of the OpMode, it will run repeatedly after clicking "Play". **/
    @Override
    public void loop() {

        pidfControllerUp.setTargetPosition(targetPosition);
        pidfControllerUp.updatePosition(robotConfig.upMotor.getCurrentPosition());

        pidfControllerDown.setTargetPosition(targetPosition);
        pidfControllerDown.updatePosition(robotConfig.downMotor.getCurrentPosition());

        double powerUp = pidfControllerUp.runPIDF() + K;
        double powerDown = pidfControllerDown.runPIDF() + K;

        if (Math.abs(robotConfig.upMotor.getCurrentPosition() - targetPosition) <= OuttakeConstants.TOLERANCE) {
            powerUp = 0;
        }

        if (Math.abs(robotConfig.downMotor.getCurrentPosition() - targetPosition) <= OuttakeConstants.TOLERANCE) {
            powerDown = 0;
        }

        robotConfig.upMotor.setPower(powerUp);
        robotConfig.downMotor.setPower(powerDown);

        updatePIDFController();

        dashboard = FtcDashboard.getInstance();
        dashboard.setTelemetryTransmissionInterval(25);
        telemetry = dashboard.getTelemetry();

        // These loop the movements of the robot
        follower.update();
        try {
            autonomousPathUpdate();
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }

        // Feedback to Driver Hub
        telemetry.addData("path state", pathState);
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.addData("timer", pathTimer.getElapsedTimeSeconds());
        telemetry.update();
  }

    /** This method is called continuously after Init while waiting for "play". **/
    @Override
    public void init_loop() {

    }

    /** This method is called once at the start of the OpMode.
     * It runs all the setup actions, including building paths and starting the path system **/
    @Override
    public void start() {
        opmodeTimer.resetTimer();
        setPathState(0);
    }

    /** We do not use this because everything should automatically disable **/
    @Override
    public void stop() {
    }

    private void updatePIDFController() {
        CustomPIDFCoefficients coefficients = new CustomPIDFCoefficients(P, I, D, F);
        pidfControllerUp = new PIDFController(coefficients);
        pidfControllerDown = new PIDFController(coefficients);
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
                ServoConstants.INTAKE_WRIST_UP,
                ServoConstants.INTAKE_WRIST_RIGHT_UP_POSITION,
                ServoConstants.INTAKE_WRIST_LEFT_UP_POSITION,
                ServoConstants.INTAKE_CLAW_OPEN_POSITION,
                ServoConstants.INTAKE_WRIST_ROT_0_DEGREES
        );
    }
}