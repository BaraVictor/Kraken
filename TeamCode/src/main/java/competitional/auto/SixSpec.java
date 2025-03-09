package competitional.auto;

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

import configurations.RobotConfig;
import constants.OuttakeConstants;
import constants.ServoConstants;
import pedroPathing.constants.FConstants;
import pedroPathing.constants.LConstants;

/**
 * This is an example auto that showcases movement and control of two servos autonomously.
 * It is a 0+4 (Specimen + Sample) bucket auto. It scores a neutral preload and then pickups 3 samples from the ground and scores them before parking.
 * There are examples of different ways to build paths.
 * A path progression method has been created and can advance based on time, position, or other factors.
 *
 * @author Baron Henderson - 20077 The Indubitables
 * @version 2.0, 11/28/2024
 */

@Autonomous(name = "6 Spec Auto ", group = "A. Competitional")
public class SixSpec extends OpMode {

    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer; //  ??

    /**
     * This is the variable where we store the state of our auto.
     * It is used by the pathUpdate method.
     */
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
    private ElapsedTime wait = new ElapsedTime();
    private boolean placeSpecimenOnChamber = true;

    public static boolean hasTransfered = false;
    public static boolean hasFinishedPickUp = false;

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

    /* Create and Define Poses + Paths
     * Poses are built with three constructors: x, y, and heading (in Radians).
     * Pedro uses 0 - 144 for x and y, with 0, 0 being on the bottom left.
     * (For Into the Deep, this would be Blue Observation Zone (0,0) to Red Observation Zone (144,144).)
     * Even though Pedro uses a different coordinate system than RR, you can convert any roadrunner pose by adding +72 both the x and y.
     * This visualizer is very easy to use to find and create paths/pathchains/poses: <https://pedro-path-generator.vercel.app/>
     * Lets assume our robot is 18 by 18 inches
     * Lets assume the Robot is facing the human player and we want to score in the bucket */

    /**
     * Start Pose of our robot
     */
    private final Pose startPose = new Pose(11, 68, toRadians(0));

    private final Pose scorePreload = new Pose(45, 68, toRadians(0));

    private final Pose sample1 = new Pose(25, 23, toRadians(0));

    private final Pose sample2 = new Pose(25, 13, toRadians(0));

    private final Pose sample3 = new Pose(25, 16, toRadians(0));


    private final Pose grab = new Pose(25, 16, toRadians(0));


    private final Pose placeControl = new Pose(20, 70, toRadians(0));

    private final Pose place1 = new Pose(40, 73, toRadians(0));

    private final Pose place2 = new Pose(40, 71, toRadians(0));

    private final Pose place3 = new Pose(40, 69, toRadians(0));

    private final Pose place4 = new Pose(40, 67, toRadians(0));

    private final Pose place5 = new Pose(40, 65, toRadians(0));

    private final Pose parkPose = new Pose(20, 30, toRadians(0));

    private PathChain park, preloadDrop, firstPickup;

    public void buildPaths() {
        preloadDrop = follower.pathBuilder()
                .addPath(new BezierLine(new Point(startPose), new Point(scorePreload)))
                .setConstantHeadingInterpolation(scorePreload.getHeading())
                .addParametricCallback(0.1, () -> lineupSpec())
                .setZeroPowerAccelerationMultiplier(4.0)
                .setPathEndTimeoutConstraint(0)
                .build();


        firstPickup = follower.pathBuilder()
                .addPath(new BezierLine(new Point(scorePreload), new Point(sample1)))
                .setLinearHeadingInterpolation(scorePreload.getHeading(), sample1.getHeading())
                .addParametricCallback(0.1, () -> follower.setMaxPower(0.8))
                .setZeroPowerAccelerationMultiplier(2.0)
                .addParametricCallback(0.5, () -> dropSpec())
                .build();

    }


    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                follower.setMaxPower(0.9);
                follower.followPath(preloadDrop, true);
                placeSpecimenOnChamber = true;
                wait.reset();
                setPathState(1);
                break;
            case 1:
                if (!follower.isBusy()) {
                    if (placeSpecimenOnChamber) {
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
                if (!placeSpecimenOnChamber) {
                    if (placeSpacimen.milliseconds() > 150) {
                        robotConfig.setOuttakeServoPositions(
                                ServoConstants.OUTTAKE_CLAW_OPEN_POSITION,
                                ServoConstants.OUTTAKE_WRIST_ROT_0_DEGREES,
                                ServoConstants.OUTTAKE_WRIST_Y_PLACE_SPECIMEN_POSITION,
                                ServoConstants.OUTTAKE_ELBOW_RIGHT_PLACE_SPECIMEN_POSITION,
                                ServoConstants.OUTTAKE_ELBOW_LEFT_PLACE_SPECIMEN_POSITION
                        );
                        targetPosition = OuttakeConstants.OUTTAKE_SECOND_SPECIMEN_BAR_RELEASE; //220
                        if(wait.seconds()>2){
                            follower.followPath(firstPickup);
                            setPathState(2);
                        }

                    }
                }
                break;

            case 2:
                if(!follower.isBusy()){
                    extendAndPickUp(0.5, ServoConstants.INTAKE_WRIST_ROT_0_DEGREES);
                    setPathState(-1);
                }


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
        telemetry.update();
    }

    @Override
    public void init() {
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
        resetServosToInit();
        buildPaths();


    }

    @Override
    public void init_loop() {
    }

    @Override
    public void start() {
        opmodeTimer.resetTimer();
        setPathState(0);
    }

    @Override
    public void stop() {
    }

    public void placeSpecimen() {

    }//?

    private void updatePIDFController() {
        CustomPIDFCoefficients coefficients = new CustomPIDFCoefficients(P, I, D, F);
        pidfController = new PIDFController(coefficients);
    }

    private void resetServosToInit() {
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
            robotConfig.upMotor.setPower(-0.9);
            robotConfig.midMotor.setPower(-0.9);
            robotConfig.downMotor.setPower(-0.9);

        } else {
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


    private void lineupSpec() {
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

    private void dropSpec() {
        targetPosition = OuttakeConstants.OUTTAKE_MIN_POSITION;
        robotConfig.setOuttakeServoPositions(
                ServoConstants.OUTTAKE_CLAW_CLOSED_POSITION,
                ServoConstants.OUTTAKE_WRIST_ROT_180_DEGREES,
                ServoConstants.OUTTAKE_WRIST_Y_PICKUP_SPECIMEN_POSITION,
                ServoConstants.OUTTAKE_ELBOW_RIGHT_SPECIMEN_POSITION,
                ServoConstants.OUTTAKE_ELBOW_LEFT_SPECIMEN_POSITION
        );
        if (robotConfig.outtakeElbowLeftServo.getPosition() < ServoConstants.OUTTAKE_ELBOW_LEFT_SPECIMEN_POSITION + 0.1)
            robotConfig.outtakeClawServo.setPosition(ServoConstants.OUTTAKE_CLAW_OPEN_POSITION);
    }

    private void pickupSpec() {
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

    private void intakeTeleOp() {
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

    private void extendAndPickup(double servoPos) {
        robotConfig.setIntakeServoPositions(
                ServoConstants.INTAKE_ELBOW_RIGHT_EXTENDED_POSITION,
                ServoConstants.INTAKE_ELBOW_LEFT_EXTENDED_POSITION,
                ServoConstants.INTAKE_WRIST_DOWN,
                ServoConstants.INTAKE_WRIST_RIGHT_DOWN_POSITION,
                ServoConstants.INTAKE_WRIST_LEFT_DOWN_POSITION,
                ServoConstants.INTAKE_CLAW_OPEN_POSITION,
                servoPos
        );
        if(robotConfig.intakeWristLeftServo.getPosition() == ServoConstants.INTAKE_WRIST_DOWN)
            robotConfig.intakeClawServo.setPosition(ServoConstants.INTAKE_CLAW_CLOSED_POSITION);

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
            if (!intaking && hoverTimer.seconds() > timer + 0.1 && hoverTimer.seconds()<timer + 0.4){ // 2 s
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

}