package org.firstinspires.ftc.teamcode.noncompetitional.auto;


import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.acmerobotics.dashboard.FtcDashboard;
import org.firstinspires.ftc.teamcode.configurations.RobotConfig;
import org.firstinspires.ftc.teamcode.constants.OuttakeConstants;
import org.firstinspires.ftc.teamcode.constants.ServoConstants;
import org.firstinspires.ftc.teamcode.pedroPathing.follower.Follower;
import org.firstinspires.ftc.teamcode.pedroPathing.localization.Pose;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierCurve;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierLine;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Path;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.PathChain;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Point;
import org.firstinspires.ftc.teamcode.pedroPathing.util.CustomPIDFCoefficients;
import org.firstinspires.ftc.teamcode.pedroPathing.util.PIDFController;
import org.firstinspires.ftc.teamcode.pedroPathing.util.Timer;

/**
 * This is an example auto that showcases movement and control of two servos autonomously.
 * There are examples of different ways to build paths.
 * A path progression method has been created and can advance based on time, position, or other factors.
 *
 * @author Baron Henderson - 20077 The Indubitables
 * @version 2.0, 11/28/2024
 */

@Config
@Autonomous(name = "SpecimenAuto", group = "A")
public class Specimen extends OpMode {

    private FtcDashboard dashboard;

    private PIDFController pidfControllerUp;
    private PIDFController pidfControllerDown;

    public static double P = 0.006;

    public static double I = 0;
    public static double D = 0;
    public static double F = 0.07;
    public static double K = 0;
    public static double targetPosition = 0;
    private ElapsedTime timer = new ElapsedTime();

    private Follower follower;
    private Timer pathTimer, opmodeTimer;
    private RobotConfig robotConfig;
    private int pathState;
    private final Pose startPose = new Pose(8, 61.5, Math.toRadians(180));  // pozitie start
    private final Pose preload = new Pose(40, 60, Math.toRadians(180));

    //sample1
    private final Pose sample1 = new Pose(34.5, 33.5, Math.toRadians(315));
    private final Pose sampleRotate1 = new Pose(31.5, 35, Math.toRadians(235));

    //sample2
    private final Pose sample2 = new Pose(34, 20.5, Math.toRadians(315));
    private final Pose sampleRotate2 = new Pose(30, 21, Math.toRadians(235));

    //sample3
    private final Pose sampleCurve3 = new Pose(38, 30, Math.toRadians(315));
    private final Pose sample3 = new Pose(33.2, 6.5, Math.toRadians(315));
    private final Pose sampleRotate3 = new Pose(25, 10, Math.toRadians(230));

    private final Pose sampleRotateControl3 = new Pose(60, 50, Math.toRadians(0));

    //specimen grab position from human player
    private final Pose grabSpecimen = new Pose(17, 25, Math.toRadians(0));

    //placing positions
    private final Pose firstDrop = new Pose(38, 62, Math.toRadians(0));
    private final Pose secondDrop = new Pose(38, 64, Math.toRadians(0));
    private final Pose thirdDrop = new Pose(38, 66, Math.toRadians(0));
    private final Pose fourthDrop = new Pose(38, 68, Math.toRadians(0));

    /* These are our Paths and PathChains that we will define in buildPaths() */
    private Path preloadDrop;
    private PathChain lineup1, score1, lineup2, lineup3, score2, score3, cycle1, cycle2, cycle3, cycle4, grab1;
    private PathChain park;

    /** Build the paths for the auto (adds, for example, constant/linear headings while doing paths)
     * It is necessary to do this so that all the paths are built before the auto starts. **/
    public void buildPaths() {
        //punem primul specimen
        preloadDrop = new Path(new BezierCurve(new Point(startPose), new Point(preload)));
        preloadDrop.setLinearHeadingInterpolation(startPose.getHeading(), preload.getHeading());

        //impinge primul spcimen spre human player
        lineup1 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(preload), new Point(sample1)))
                .setLinearHeadingInterpolation(preload.getHeading(), sample1.getHeading())
                .build();

        score1  = follower.pathBuilder()
                .addPath(new BezierLine(new Point(sample1), new Point(sampleRotate1)))
                .setLinearHeadingInterpolation(sample1.getHeading(), sampleRotate1.getHeading())
                .build();

        lineup2 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(sampleRotate1), new Point(sample2)))
                .setLinearHeadingInterpolation(sampleRotate1.getHeading(), sample2.getHeading())
                .build();

        score2 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(sample2), new Point(sampleRotate2)))
                .setLinearHeadingInterpolation(sample2.getHeading(), sampleRotate2.getHeading())
                .build();

        lineup3 = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(sampleRotate2), new Point(sampleCurve3), new Point(sample3)))
                .setLinearHeadingInterpolation(sampleRotate2.getHeading(), sample3.getHeading())
                .build();

        score3 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(sample3), new Point(sampleRotate3)))
                .setLinearHeadingInterpolation(sample3.getHeading(), sampleRotate3.getHeading())
                .build();

        // time to start cycling

        grab1 = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(sampleRotate3), new Point(sampleRotateControl3), new Point(grabSpecimen)))
                .setLinearHeadingInterpolation(sampleRotate3.getHeading(), grabSpecimen.getHeading())
                .build();

        cycle1 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(grabSpecimen), new Point(firstDrop)))
                .setLinearHeadingInterpolation(grabSpecimen.getHeading(), firstDrop.getHeading())
                .build();


        cycle2 = follower.pathBuilder().addPath(new BezierLine(new Point(firstDrop), new Point(grabSpecimen)))
                .setLinearHeadingInterpolation(firstDrop.getHeading(), grabSpecimen.getHeading())
                .addPath(new BezierLine(new Point(grabSpecimen), new Point(secondDrop)))
                .setLinearHeadingInterpolation(grabSpecimen.getHeading(), secondDrop.getHeading())
                .build();


        cycle3 = follower.pathBuilder().addPath(new BezierLine(new Point(secondDrop), new Point(grabSpecimen)))
                .setLinearHeadingInterpolation(secondDrop.getHeading(), grabSpecimen.getHeading())
                .addPath(new BezierLine(new Point(grabSpecimen), new Point(thirdDrop)))
                .setLinearHeadingInterpolation(grabSpecimen.getHeading(), thirdDrop.getHeading())
                .build();

        cycle4 = follower.pathBuilder().addPath(new BezierLine(new Point(thirdDrop), new Point(grabSpecimen)))
                .setLinearHeadingInterpolation(thirdDrop.getHeading(), grabSpecimen.getHeading())
                .addPath(new BezierLine(new Point(grabSpecimen), new Point(fourthDrop)))
                .setLinearHeadingInterpolation(grabSpecimen.getHeading(), fourthDrop.getHeading())
                .build();

        park = follower.pathBuilder()
                .addPath(new BezierLine(new Point(fourthDrop), new Point(grabSpecimen)))
                .setLinearHeadingInterpolation(fourthDrop.getHeading(), grabSpecimen.getHeading())
                .build();

    }
    /** This switch is called continuously and runs the pathing, at certain points, it triggers the action state.
     * Everytime the switch changes case, it will reset the timer. (This is because of the setPathState() method)
     * The followPath() function sets the follower to run the specific path, but does NOT wait for it to finish before moving on. */

    /* You could check for
                - Follower State: "if(!follower.isBusy() {}" (Though, I don't recommend this because it might not return due to holdEnd
                - Time: "if(pathTimer.getElapsedTimeSeconds() > 1) {}"
                - Robot Position: "if(follower.getPose().getX() > 36) {}"
                */

    private boolean hover = false;
    private boolean deposited = false;
    private boolean hovering = false;
    private boolean intaking = false;
    private boolean transfer = false;
    private boolean hasTurned = false;
    private boolean closed = false;
    private boolean opened = false;
    private boolean hasTransfered = false;

    private ElapsedTime closingTimer = new ElapsedTime();
    private ElapsedTime retractTimer = new ElapsedTime();
    private ElapsedTime hoverTimer = new ElapsedTime();
    private ElapsedTime intakeTimer = new ElapsedTime();
    private ElapsedTime turningTimer = new ElapsedTime();
    private ElapsedTime revolutTimer = new ElapsedTime();
    private ElapsedTime launchingTimer = new ElapsedTime();
    private ElapsedTime turningTimer2 = new ElapsedTime();
    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                follower.followPath(preloadDrop);
                setPathState(1);
                break;

            case 1:
                if (follower.getPose().getX() > (preload.getX() - 1) && follower.getPose().getY() > (preload.getY() - 1)) {
                    follower.followPath(lineup1, true);
                    turningTimer.reset();
                    closingTimer.reset();
                    setPathState(2);
                }
                break;
            case 2:
                if (follower.getPose().getX() > (sample1.getX() - 1) && follower.getPose().getY() > (sample1.getY() - 1)) {
                    hasTurned = true;
                }
                if (!hovering && Math.toDegrees(follower.getPose().getHeading()) > 313 && hasTurned && turningTimer.seconds() > 1.4) {
                    robotConfig.setIntakeServoPositions(
                            ServoConstants.INTAKE_ELBOW_RIGHT_EXTENDED_POSITION,
                            ServoConstants.INTAKE_ELBOW_LEFT_EXTENDED_POSITION,
                            ServoConstants.INTAKE_WRIST_DOWN,
                            ServoConstants.INTAKE_WRIST_RIGHT_HOVER_POSITION,
                            ServoConstants.INTAKE_WRIST_LEFT_HOVER_POSITION,
                            ServoConstants.INTAKE_CLAW_OPEN_POSITION,
                            ServoConstants.INTAKE_WRIST_ROT_45_DEGREES
                    );
                    hoverTimer.reset();
                }
                if (robotConfig.intakeElbowRightServo.getPosition() == ServoConstants.INTAKE_ELBOW_RIGHT_EXTENDED_POSITION) {
                    hovering = true;
                    if (!intaking && hoverTimer.seconds() > 1) {
                        robotConfig.setIntakeServoPositions(
                                ServoConstants.INTAKE_ELBOW_RIGHT_EXTENDED_POSITION,
                                ServoConstants.INTAKE_ELBOW_LEFT_EXTENDED_POSITION,
                                ServoConstants.INTAKE_WRIST_DOWN,
                                ServoConstants.INTAKE_WRIST_RIGHT_DOWN_POSITION,
                                ServoConstants.INTAKE_WRIST_LEFT_DOWN_POSITION,
                                ServoConstants.INTAKE_CLAW_OPEN_POSITION,
                                ServoConstants.INTAKE_WRIST_ROT_45_DEGREES);
                        intaking = true;
                    } else {
                        if (hoverTimer.seconds() > 1.3 && !hover) {
                            robotConfig.setIntakeServoPositions(
                                    ServoConstants.INTAKE_ELBOW_RIGHT_EXTENDED_POSITION,
                                    ServoConstants.INTAKE_ELBOW_LEFT_EXTENDED_POSITION,
                                    ServoConstants.INTAKE_WRIST_DOWN,
                                    ServoConstants.INTAKE_WRIST_RIGHT_DOWN_POSITION,
                                    ServoConstants.INTAKE_WRIST_LEFT_DOWN_POSITION,
                                    ServoConstants.INTAKE_CLAW_CLOSED_POSITION,
                                    ServoConstants.INTAKE_WRIST_ROT_45_DEGREES
                            );
                            if (robotConfig.intakeClawServo.getPosition() > ServoConstants.INTAKE_CLAW_CLOSED_POSITION - 0.1) {
                                hover = true;
                            }
                            if (hover && closingTimer.seconds() > 1.5) {
                                robotConfig.setIntakeServoPositions(
                                        ServoConstants.INTAKE_ELBOW_RIGHT_EXTENDED_POSITION,
                                        ServoConstants.INTAKE_ELBOW_LEFT_EXTENDED_POSITION,
                                        ServoConstants.INTAKE_WRIST_DOWN,
                                        ServoConstants.INTAKE_WRIST_RIGHT_HOVER_POSITION,
                                        ServoConstants.INTAKE_WRIST_LEFT_HOVER_POSITION,
                                        ServoConstants.INTAKE_CLAW_CLOSED_POSITION,
                                        ServoConstants.INTAKE_WRIST_ROT_45_DEGREES
                                );
                                follower.followPath(score1, true);
                                hasTurned = false;
                                launchingTimer.reset();
                                setPathState(3);
                            }
                        }
                    }
                }
                break;
            case 3:
                if (follower.getPose().getX() > (sampleRotate1.getX() - 1) && follower.getPose().getY() > (sampleRotate1.getY() - 1)) {
                    hasTurned = true;
                }
                if (hasTurned && Math.toDegrees(follower.getPose().getHeading()) < 260) {
                    robotConfig.intakeClawServo.setPosition(ServoConstants.INTAKE_CLAW_OPEN_POSITION);
                    follower.followPath(lineup2, true);
                    hasTurned = false;
                    hovering = false;
                    hover=false;
                    turningTimer.reset();
                    intaking = false;
                    closingTimer.reset();
                    hoverTimer.reset();
                    setPathState(4);
                }
                break;
            case 4:
                if (follower.getPose().getX() > (sample2.getX() - 1) && follower.getPose().getY() > (sample2.getY() - 1)) {
                    hasTurned = true;
                }
                    hovering = true;
                    if (!intaking && hoverTimer.seconds() > 2) {
                        robotConfig.setIntakeServoPositions(
                                ServoConstants.INTAKE_ELBOW_RIGHT_EXTENDED_POSITION,
                                ServoConstants.INTAKE_ELBOW_LEFT_EXTENDED_POSITION,
                                ServoConstants.INTAKE_WRIST_DOWN,
                                ServoConstants.INTAKE_WRIST_RIGHT_DOWN_POSITION,
                                ServoConstants.INTAKE_WRIST_LEFT_DOWN_POSITION,
                                ServoConstants.INTAKE_CLAW_OPEN_POSITION,
                                ServoConstants.INTAKE_WRIST_ROT_45_DEGREES);
                        intaking = true;
                    } else {
                        if (hoverTimer.seconds() > 2.4 && !hover) {
                            robotConfig.setIntakeServoPositions(
                                    ServoConstants.INTAKE_ELBOW_RIGHT_EXTENDED_POSITION,
                                    ServoConstants.INTAKE_ELBOW_LEFT_EXTENDED_POSITION,
                                    ServoConstants.INTAKE_WRIST_DOWN,
                                    ServoConstants.INTAKE_WRIST_RIGHT_DOWN_POSITION,
                                    ServoConstants.INTAKE_WRIST_LEFT_DOWN_POSITION,
                                    ServoConstants.INTAKE_CLAW_CLOSED_POSITION,
                                    ServoConstants.INTAKE_WRIST_ROT_45_DEGREES
                            );
                            if (robotConfig.intakeClawServo.getPosition() > ServoConstants.INTAKE_CLAW_CLOSED_POSITION - 0.1) {
                                hover = true;
                            }
                            if (hover && closingTimer.seconds() > 1.5) {
                                robotConfig.setIntakeServoPositions(
                                        ServoConstants.INTAKE_ELBOW_RIGHT_EXTENDED_POSITION,
                                        ServoConstants.INTAKE_ELBOW_LEFT_EXTENDED_POSITION,
                                        ServoConstants.INTAKE_WRIST_DOWN,
                                        ServoConstants.INTAKE_WRIST_RIGHT_HOVER_POSITION,
                                        ServoConstants.INTAKE_WRIST_LEFT_HOVER_POSITION,
                                        ServoConstants.INTAKE_CLAW_CLOSED_POSITION,
                                        ServoConstants.INTAKE_WRIST_ROT_45_DEGREES
                                );
                                follower.followPath(score2, true);
                                hasTurned = false;
                                launchingTimer.reset();
                                setPathState(5);
                            }
                        }
                    }
                break;
            case 5:
                if (follower.getPose().getX() > (sampleRotate2.getX() - 1) && follower.getPose().getY() > (sampleRotate2.getY() - 1)) {
                    hasTurned = true;
                }
                if (hasTurned && Math.toDegrees(follower.getPose().getHeading()) < 260) {
                    robotConfig.intakeClawServo.setPosition(ServoConstants.INTAKE_CLAW_OPEN_POSITION);
                    follower.followPath(lineup3, true);
                    hasTurned = false;
                    hovering = false;
                    hover=false;
                    turningTimer.reset();
                    intaking = false;
                    closingTimer.reset();
                    hoverTimer.reset();
                    setPathState(6);
                }
                break;
            case 6:
                if (follower.getPose().getX() > (sample2.getX() - 1) && follower.getPose().getY() > (sample2.getY() - 1)) {
                    hasTurned = true;
                }
                hovering = true;
                if (!intaking && hoverTimer.seconds() > 2) {
                    robotConfig.setIntakeServoPositions(
                            ServoConstants.INTAKE_ELBOW_RIGHT_EXTENDED_POSITION,
                            ServoConstants.INTAKE_ELBOW_LEFT_EXTENDED_POSITION,
                            ServoConstants.INTAKE_WRIST_DOWN,
                            ServoConstants.INTAKE_WRIST_RIGHT_DOWN_POSITION,
                            ServoConstants.INTAKE_WRIST_LEFT_DOWN_POSITION,
                            ServoConstants.INTAKE_CLAW_OPEN_POSITION,
                            ServoConstants.INTAKE_WRIST_ROT_45_DEGREES);
                    intaking = true;
                } else {
                    if (hoverTimer.seconds() > 2.4 && !hover) {
                        robotConfig.intakeClawServo.setPosition(ServoConstants.INTAKE_CLAW_CLOSED_POSITION);
                        if (robotConfig.intakeClawServo.getPosition() > ServoConstants.INTAKE_CLAW_CLOSED_POSITION - 0.1) {
                            hover = true;
                        }
                    }
                    if (hover && closingTimer.seconds() > 2.5) {
                        robotConfig.setIntakeServoPositions(
                                ServoConstants.INTAKE_ELBOW_RIGHT_SEMIRETRACTED_POSITION,
                                ServoConstants.INTAKE_ELBOW_LEFT_SEMIRETRACTED_POSITION,
                                ServoConstants.INTAKE_WRIST_DOWN,
                                ServoConstants.INTAKE_WRIST_RIGHT_HOVER_POSITION,
                                ServoConstants.INTAKE_WRIST_LEFT_HOVER_POSITION,
                                ServoConstants.INTAKE_CLAW_CLOSED_POSITION,
                                ServoConstants.INTAKE_WRIST_ROT_45_DEGREES
                        );
                        follower.followPath(score3, true);
                        hasTurned = false;
                        launchingTimer.reset();
                        setPathState(7);
                    }
                }
                break;
            case 7:
                if (follower.getPose().getX() > (sampleRotate3.getX() - 1) && follower.getPose().getY() > (sampleRotate3.getY() - 1)) {
                    hasTurned = true;
                    robotConfig.setIntakeServoPositions(
                            ServoConstants.INTAKE_ELBOW_RIGHT_RETRACTED_POSITION,
                            ServoConstants.INTAKE_ELBOW_LEFT_RETRACTED_POSITION,
                            ServoConstants.INTAKE_WRIST_MID,
                            ServoConstants.INTAKE_WRIST_RIGHT_UP_POSITION,
                            ServoConstants.INTAKE_WRIST_LEFT_UP_POSITION,
                            ServoConstants.INTAKE_CLAW_OPEN_POSITION,
                            ServoConstants.INTAKE_WRIST_ROT_0_DEGREES
                    );
                    robotConfig.setOuttakeServoPositions(
                            ServoConstants.OUTTAKE_CLAW_OPEN_POSITION,
                            ServoConstants.OUTTAKE_WRIST_ROT_0_DEGREES,
                            ServoConstants.OUTTAKE_WRIST_Y_PICKUP_SPECIMEN_POSITION,
                            ServoConstants.OUTTAKE_ELBOW_RIGHT_SPECIMEN_POSITION,
                            ServoConstants.OUTTAKE_ELBOW_LEFT_SPECIMEN_POSITION
                    );
                }
                if (hasTurned && Math.toDegrees(follower.getPose().getHeading()) < 240) {
                    robotConfig.intakeClawServo.setPosition(ServoConstants.INTAKE_CLAW_OPEN_POSITION);
                    follower.followPath(grab1, true);
                    hasTurned = false;
                    hovering = false;
                    hover=false;
                    turningTimer.reset();
                    intaking = false;
                    closingTimer.reset();
                    hoverTimer.reset();
                    setPathState(8);
                }
                break;
                case 8:
                    if(follower.getPose().getX() > (grabSpecimen.getX() - 1) && follower.getPose().getY() > (grabSpecimen.getY() - 1)){
                        follower.followPath(cycle1, true);
                        setPathState(9);
                    }
                    break;
                case 9:
                    if(follower.getPose().getX() > (firstDrop.getX() - 1) && follower.getPose().getY() > (firstDrop.getY() - 1)){
                        follower.followPath(grab1, true);
                        setPathState(10);
                    }
                    break;
                case 10:
                    if(follower.getPose().getX() > (grabSpecimen.getX() - 1) && follower.getPose().getY() > (grabSpecimen.getY() - 1)){
                        follower.followPath(cycle2, true);
                        setPathState(11);
                    }
                    break;
                case 11:
                    if(follower.getPose().getX() > (secondDrop.getX() - 1) && follower.getPose().getY() > (secondDrop.getY() - 1)){
                        follower.followPath(grab1, true);
                        setPathState(12);
                    }
                    break;
                    case 12:
                    if(follower.getPose().getX() > (grabSpecimen.getX() - 1) && follower.getPose().getY() > (grabSpecimen.getY() - 1)){
                        follower.followPath(cycle3, true);
                        setPathState(13);
                    }
                    break;
                case 13:
                    if(follower.getPose().getX() > (thirdDrop.getX() - 1) && follower.getPose().getY() > (thirdDrop.getY() - 1)){
                        follower.followPath(grab1, true);
                        setPathState(14);
                    }
                    break;
        }
    }



    /** These change the states of the paths and actions
     * It will also reset the timers of the individual switches **/
    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
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

        // These loop the movements of the robot
        follower.update();
        autonomousPathUpdate();

        // Feedback to Driver Hub
        dashboard = FtcDashboard.getInstance();
        dashboard.setTelemetryTransmissionInterval(25); // Set telemetry update frequency
        telemetry = dashboard.getTelemetry();
        telemetry.addData("path state", pathState);
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", (follower.getPose().getHeading()*180)/Math.PI);
        telemetry.addData("timer", timer.seconds());
        telemetry.addData("launchingTimer", launchingTimer.seconds());
        telemetry.addData("Target Position", targetPosition);
        telemetry.addData("Left Motor Position", robotConfig.upMotor.getCurrentPosition());
        telemetry.addData("Right Motor Position", robotConfig.downMotor.getCurrentPosition());
        telemetry.addData("Left Motor Error", robotConfig.upMotor.getCurrentPosition() - targetPosition);
        telemetry.addData("Right Motor Error", robotConfig.downMotor.getCurrentPosition() - targetPosition);
        telemetry.update();
    }

    /** This method is called once at the init of the OpMode. **/
    @Override
    public void init() {
        dashboard = FtcDashboard.getInstance();
        dashboard.setTelemetryTransmissionInterval(25); // Set telemetry update frequency
        telemetry = dashboard.getTelemetry();
        robotConfig = new RobotConfig(hardwareMap);

        robotConfig.upMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robotConfig.upMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robotConfig.upMotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        robotConfig.upMotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        robotConfig.upMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        robotConfig.upMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        updatePIDFController();

        pathTimer = new Timer();
        opmodeTimer = new Timer();

        opmodeTimer.resetTimer();

        follower = new Follower(hardwareMap);
        follower.setStartingPose(startPose);

        resetServosToInit();

        buildPaths();
    }

    /** This method is called continuously after Init while waiting for "play". **/
    @Override
    public void init_loop() {}

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
    private void updatePIDFController () {
        CustomPIDFCoefficients coefficients = new CustomPIDFCoefficients(P, I, D, F);
        pidfControllerUp = new PIDFController(coefficients);
        pidfControllerDown = new PIDFController(coefficients);
    }

    private void resetServosToInit () {
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