package org.firstinspires.ftc.teamcode.noncompetitional.auto;

import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
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

@Autonomous(name = "🐙 SampleAutoColor 🐙", group = "Auto")
public class SampleColour extends OpMode {

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
    private boolean hasTransfered = false;

    private RobotConfig robotConfig;

    private PIDFController pidfControllerUp;
    private PIDFController pidfControllerDown;

    public static double P = 0.006;

    public static double I = 0;
    public static double D = 0;
    public static double F = 0.07;
    public static double K = 0;
    public static double targetPosition = 0;

    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;
    private int pathState;

    private ColorSensor colorSensor;    // REV Color Sensor V3
    private DistanceSensor distanceSensor;
    /*
    TO Do:
    Scurtat timere
    Meshup paths-slidere (actiuni)
    Finalizare parcare

    Done:
    Schimbate pozitiile
    Outtake se da peste cap dupa ce ajunge sus
     */


    private final Pose score = new Pose(21, 128, Math.toRadians(318));
    private final Pose scorePos2 = new Pose(20.5, 129, Math.toRadians(318));
    private final Pose scorePos3 = new Pose(21, 129, Math.toRadians(318));
    private final Pose startPose = new Pose(7, 112, Math.toRadians(270));
    private final Pose preload = new Pose(20, 126, Math.toRadians(318));
    private final Pose sample1 = new Pose(22.5, 126, Math.toRadians(357));
    private final Pose sample2 = new Pose(22.5, 131, Math.toRadians(6.5));
    private final Pose sample3 = new Pose(23, 139, Math.toRadians(15));
    private final Pose park = new Pose(80, 95, Math.toRadians(0));
    private final Pose parkFinal = new Pose(80, 10 , Math.toRadians(90));


    //test

    private final Pose one = new Pose(65, 135, Math.toRadians(180));

    private final Pose two = new Pose(65, 100, Math.toRadians(180));


    private Path scorePreload;
    private PathChain firstPickup, secondPickup, thirdPickup, parkPath, score1, score2, score3, parkPsuh, test;

    public void buildPaths() {

        test = follower.pathBuilder()
                .addPath(new BezierLine(new Point(one), new Point(two)))
                .setLinearHeadingInterpolation(one.getHeading(), two.getHeading())
                .build();

        /* This is our grabPickup1 PathChain. We are using a single path with a BezierLine, which is a straight line. */


        /* This is our scorePickup2 PathChain. We are using a single path with a BezierLine, which is a straight line. */

        /* This is our park path. We are using a BezierCurve with 3 points, which is a curved line that is curved based off of the control point */
    }

    /**
     * This switch is called continuously and runs the pathing, at certain points, it triggers the action state.
     * Everytime the switch changes case, it will reset the timer. (This is because of the setPathState() method)
     * The followPath() function sets the follower to run the specific path, but does NOT wait for it to finish before moving on.
     */
    public void autonomousPathUpdate() {
        colorSensor = hardwareMap.get(ColorSensor.class, "colorSensor");
        distanceSensor = hardwareMap.get(DistanceSensor.class, "colorSensor"); // Distance is part of the same device

        float[] hsvValues = {0F, 0F, 0F};
        switch (pathState) {
            case 0:


                robotConfig.setIntakeServoPositions(
                        ServoConstants.INTAKE_ELBOW_RIGHT_EXTENDED_POSITION,
                        ServoConstants.INTAKE_ELBOW_LEFT_EXTENDED_POSITION,
                        ServoConstants.INTAKE_WRIST_DOWN,
                        ServoConstants.INTAKE_WRIST_RIGHT_HOVER_POSITION,
                        ServoConstants.INTAKE_WRIST_LEFT_HOVER_POSITION,
                        ServoConstants.INTAKE_CLAW_OPEN_POSITION,
                        ServoConstants.INTAKE_WRIST_ROT_0_DEGREES
                );

                Color.RGBToHSV(
                        (int) (colorSensor.red() * 255.0 / 1023.0),
                        (int) (colorSensor.green() * 255.0 / 1023.0),
                        (int) (colorSensor.blue() * 255.0 / 1023.0),
                        hsvValues
                );

                // Extract HSV components
                float hue = hsvValues[0]; // Hue in degrees (0 - 360)
                float saturation = hsvValues[1]; // Saturation (0.0 - 1.0)
                float value = hsvValues[2]; // Value (brightness) (0.0 - 1.0)

                // Detect yellow, blue, and red based on HSV ranges
                boolean isYellow = (hue >= 70 && hue <= 90) && (saturation >= 0.4) && (value >= 0.06);
                boolean isBlue = (hue >= 190 && hue <= 260) && (saturation >= 0.4) && (value >= 0.04);
                boolean isRed = ((hue >= 0 && hue <= 50) || (hue >= 340 && hue <= 360)) && (saturation >= 0.3) && (value >= 0.04);

                // Measure distance in cm
                double distanceCm = distanceSensor.getDistance(DistanceUnit.CM);

                // Display data on Driver Station
                telemetry.addData("Hue", hue);
                telemetry.addData("Saturation", saturation);
                telemetry.addData("Value", value);
                telemetry.addData("Is Yellow?", isYellow ? "Yes" : "No");
                telemetry.addData("Is Blue?", isBlue ? "Yes" : "No");
                telemetry.addData("Is Red?", isRed ? "Yes" : "No");
                telemetry.addData("Distance (cm)", String.format("%.2f cm", distanceCm));
                telemetry.update();

        }
    }
    /** These change the states of the paths and actions
     * It will also reset the timers of the individual switches **/
    public void setPathState ( int pState){
        pathState = pState;
        pathTimer.resetTimer();
    }
    @Override
    public void init () {
        robotConfig = new RobotConfig(hardwareMap);
        robotConfig.upMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robotConfig.upMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robotConfig.upMotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        robotConfig.upMotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        robotConfig.upMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        robotConfig.upMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        updatePIDFController();

        telemetry.addLine("Outtake Sliders Initialized with PIDF and Joystick Control");
        pathTimer = new Timer();
        opmodeTimer = new Timer();

        opmodeTimer.resetTimer();

        follower = new Follower(hardwareMap);
        follower.setStartingPose(startPose);

        buildPaths();

        resetServosToInit();

        telemetry.addData("Status", "Initialized");
        telemetry.addLine("Motors have been reset");
        telemetry.update();
    }
    /** This is the main loop of the OpMode, it will run repeatedly after clicking "Play". **/
    @Override
    public void loop () {


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
        telemetry.addData("path state", pathState);
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", Math.toDegrees(follower.getPose().getHeading()));
        telemetry.addData("timer", pathTimer.getElapsedTimeSeconds());
        telemetry.addData("retract timer", retractTimer.seconds());
        telemetry.addData("hover timer", hoverTimer.seconds());
        telemetry.addData("slider pos", robotConfig.upMotor.getCurrentPosition());
        telemetry.addData("target pos", targetPosition);
        telemetry.addData("deposited", deposited);
        telemetry.addData("intakeTimer", intakeTimer.seconds());
        telemetry.addData("transfer", transfer);
        telemetry.addData("has turned", hasTurned);
        telemetry.update();
    }

    /** This method is called continuously after Init while waiting for "play". **/
    @Override
    public void init_loop () {
    }

    /** This method is called once at the start of the OpMode.
     * It runs all the setup actions, including building paths and starting the path system **/
    @Override
    public void start () {
        opmodeTimer.resetTimer();
        setPathState(0);
    }

    /** We do not use this because everything should automatically disable **/
    @Override
    public void stop () {
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