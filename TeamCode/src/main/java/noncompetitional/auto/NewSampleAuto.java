package noncompetitional.auto;

import static java.lang.Math.toRadians;

import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.Path;
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

@Autonomous(name = "New Example Auto ", group = "Examples")
public class NewSampleAuto extends OpMode {

    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer; //  ??

    /**
     * This is the variable where we store the state of our auto.
     * It is used by the pathUpdate method.
     */
    private int pathState;
    private RobotConfig robotConfig;
    private PIDFController pidfController;

    public static double P = 0.0128;
    public static double I = 0;
    public static double D = 0.00005;
    public static double F = 0.01;
    public static double K = 0;
    public static double targetPosition = 0;
    private boolean areSlidesDown = false;

    /* Create and Define Poses + Paths
     * Poses are built with three constructors: x, y, and heading (in Radians).
     * Pedro uses 0 - 144 for x and y, with 0, 0 being on the bottom left.
     * (For Into the Deep, this would be Blue Observation Zone (0,0) to Red Observation Zone (144,144).)
     * Even though Pedro uses a different coordinate system than RR, you can convert any roadrunner pose by adding +72 both the x and y.
     * This visualizer is very easy to use to find and create paths/pathchains/poses: <https://pedro-path-generator.vercel.app/>
     * Lets assume our robot is 18 by 18 inches
     * Lets assume the Robot is facing the human player and we want to score in the bucket */


    private final Pose startPose = new Pose(7.5, 111, toRadians(270));

    private final Pose scorePose = new Pose(20, 129, toRadians(315));

    private final Pose sample1 = new Pose(24, 121, toRadians(0));

    private final Pose sample2 = new Pose(24, 132, toRadians(0));

    private final Pose sample3 = new Pose(31, 125, toRadians(48));


    /* These are our Paths and PathChains that we will define in buildPaths() */
    private Path subPickup; //?
    private PathChain preloadDrop, firstPickup, secondPickup, thirdPickup, fourthPickup, drop1, drop2; //?

    public void buildPaths() {
        preloadDrop = follower.pathBuilder()
                .addPath(new Path((new BezierLine(new Point(startPose), new Point(scorePose)))))
                .setLinearHeadingInterpolation(startPose.getHeading(), scorePose.getHeading())
                .setPathEndTimeoutConstraint(0)
//                .addParametricCallback(0.7, () -> liftSlides())
//                .addParametricCallback(1,() -> liftScore())
                .build();

        firstPickup = follower.pathBuilder()
//                .addParametricCallback(0.3,() -> downSlides())
                .addPath(new BezierLine(new Point(scorePose), new Point(sample1)))
                .setLinearHeadingInterpolation(scorePose.getHeading(),sample1.getHeading())
                .setPathEndTimeoutConstraint(0)
                .build();

        drop1 = follower.pathBuilder()
                .addPath(new Path((new BezierLine(new Point(sample1), new Point(scorePose)))))
                .setLinearHeadingInterpolation(sample1.getHeading(), scorePose.getHeading())

                .setPathEndTimeoutConstraint(0)
//                .addParametricCallback(0.7, () -> liftSlides())
//                .addParametricCallback(1,() -> liftScore())
                .build();

        secondPickup = follower.pathBuilder()
//                .addParametricCallback(0.3,() -> downSlides())
                .addPath(new BezierLine(new Point(scorePose), new Point(sample2)))
                .setLinearHeadingInterpolation(scorePose.getHeading(),sample2.getHeading())
                .setPathEndTimeoutConstraint(0)
                .build();

    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                follower.setMaxPower(0.8);
                follower.followPath(preloadDrop);
                //liftSlides();
                setPathState(1);
                break;
            case 1:
                if (!follower.isBusy()) {
                    follower.setMaxPower(0.5);
                    follower.followPath(firstPickup);
                    //liftSlides();
                    setPathState(2);
                }
                break;
            case 2:
                if (!follower.isBusy()) {
                    follower.followPath(drop1);
                    //liftSlides();
                    setPathState(3);
                }
                break;
            case 3:
                if (!follower.isBusy()) {
                    follower.followPath(secondPickup);
                    //liftSlides();
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

    /**
     * This is the main loop of the OpMode, it will run repeatedly after clicking "Play".
     **/
    @Override
    public void loop() {

        // These loop the movements of the robot
        follower.update();
        autonomousPathUpdate();
        controlPID();
        // Feedback to Driver Hub
        telemetry.addData("path state", pathState);
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.addData("isBusy", follower.isBusy());
        telemetry.update();
    }

    /**
     * This method is called once at the init of the OpMode.
     **/
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
                ServoConstants.OUTTAKE_WRIST_Y_TRANSFER_POSITION,
                ServoConstants.OUTTAKE_ELBOW_RIGHT_PICKUP_POSITION,
                ServoConstants.OUTTAKE_ELBOW_LEFT_PICKUP_POSITION
        );
        robotConfig.setIntakeServoPositions(
                ServoConstants.INTAKE_ELBOW_RIGHT_RETRACTED_POSITION,
                ServoConstants.INTAKE_ELBOW_LEFT_RETRACTED_POSITION,
                ServoConstants.INTAKE_WRIST_UP,
                ServoConstants.INTAKE_WRIST_RIGHT_UP_POSITION - 0.1,
                ServoConstants.INTAKE_WRIST_LEFT_UP_POSITION - 0.1,
                ServoConstants.INTAKE_CLAW_OPEN_POSITION,
                ServoConstants.INTAKE_WRIST_ROT_0_DEGREES
        );
    }

    private void controlPID() {
        pidfController.setTargetPosition(targetPosition);
        pidfController.updatePosition(robotConfig.upMotor.getCurrentPosition());

        double powerUp = pidfController.runPIDF() + K;
//            double powerDown = pidfControllerDown.runPIDF() + K;


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

    private ElapsedTime placeTimer = new ElapsedTime();

    private ElapsedTime slideTimer = new ElapsedTime();

    private void liftSlides() {
            robotConfig.setOuttakeServoPositions(
                    ServoConstants.OUTTAKE_CLAW_CLOSED_POSITION,
                    ServoConstants.OUTTAKE_WRIST_ROT_0_DEGREES,
                    ServoConstants.OUTTAKE_WRIST_Y_PLACE_SPECIMEN_POSITION,
                    ServoConstants.OUTTAKE_ELBOW_RIGHT_PLACE_SPECIMEN_POSITION,
                    ServoConstants.OUTTAKE_ELBOW_LEFT_PLACE_SPECIMEN_POSITION
            );
            targetPosition = OuttakeConstants.OUTTAKE_TOP_SAMPLE_BOX;
        }


    private void liftScore() {
            robotConfig.setOuttakeServoPositions(
                    ServoConstants.OUTTAKE_CLAW_OPEN_POSITION,
                    ServoConstants.OUTTAKE_WRIST_ROT_0_DEGREES,
                    ServoConstants.OUTTAKE_WRIST_Y_PLACE_SPECIMEN_POSITION,
                    ServoConstants.OUTTAKE_ELBOW_RIGHT_PLACE_SPECIMEN_POSITION,
                    ServoConstants.OUTTAKE_ELBOW_LEFT_PLACE_SPECIMEN_POSITION
            );
        }

        private void downSlides() {
        targetPosition = OuttakeConstants.OUTTAKE_MIN_POSITION;
        }

        private void extendIntake() {
            targetPosition = OuttakeConstants.OUTTAKE_MIN_POSITION;
            robotConfig.setIntakeServoPositions(
                    ServoConstants.INTAKE_ELBOW_RIGHT_EXTENDED_POSITION,
                    ServoConstants.INTAKE_ELBOW_LEFT_EXTENDED_POSITION,
                    ServoConstants.INTAKE_WRIST_DOWN,
                    ServoConstants.INTAKE_WRIST_RIGHT_DOWN_POSITION,
                    ServoConstants.INTAKE_WRIST_LEFT_DOWN_POSITION,
                    ServoConstants.INTAKE_CLAW_OPEN_POSITION,
                    ServoConstants.INTAKE_WRIST_ROT_0_DEGREES
            );
        }

        public void transfer() {
            robotConfig.setOuttakeServoPositions(
                    ServoConstants.OUTTAKE_CLAW_OPEN_POSITION,
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
                    ServoConstants.INTAKE_CLAW_CLOSED_POSITION,
                    ServoConstants.INTAKE_WRIST_ROT_0_DEGREES
            );
        }

    private Runnable callBackSetMaxPower = new Runnable() {
        @Override
        public void run() {
            follower.setMaxPower(0.5);
        }
    };

    private Runnable callBackSetMaxPower2 = new Runnable() {
        @Override
        public void run() {
            follower.setMaxPower(0.9);
        }
    };

    private Runnable lift = new Runnable() {
        @Override
        public void run() {
            follower.setMaxPower(0.9);
        }
    };

}