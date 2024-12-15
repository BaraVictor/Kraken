package org.firstinspires.ftc.teamcode.noncompetitional.auto;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import  com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.pedroPathing.follower.*;
import org.firstinspires.ftc.teamcode.pedroPathing.localization.Pose;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierCurve;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierLine;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Path;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.PathChain;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Point;
import org.firstinspires.ftc.teamcode.pedroPathing.util.Timer;

/**
 * This is an example auto that showcases movement and control of two servos autonomously.
 * It is a 0+4 (Specimen + Sample) bucket auto. It scores a neutral preload and then pickups 3 samples from the ground and scores them before parking.
 * There are examples of different ways to build paths.
 * A path progression method has been created and can advance based on time, position, or other factors.
 *
 * @author Baron Henderson - 20077 The Indubitables
 * @version 2.0, 11/28/2024
 */

@Autonomous(name = "Example Specimen Auto Red", group = "Exemple autonomie")
public class SimpleSpecimen extends OpMode {

    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;

    /** This is the variable where we store the state of our auto.
     * It is used by the pathUpdate method. */
    private int pathState;

    /** This is our claw subsystem.
     * We call its methods to manipulate the servos that it has within the subsystem. */

//    public ClawSubsystem claw;

    /** Create and Define Poses + Paths
     * Poses are built with three constructors: x, y, and heading (in Radians).
     * Pedro uses 0 - 144 for x and y, with 0, 0 being on the bottom left.
     * (For Into the Deep, this would be Blue Observation Zone (0,0) to Red Observation Zone (144,144).)
     * Even though Pedro uses a different coordinate system than RR, you can convert any roadrunner pose by adding +72 both the x and y.
     * This visualizer is very easy to use to find and create paths/pathchains/poses: <https://pedro-path-generator.vercel.app/>
     * Lets assume our robot is 18 by 18 inches
     * Lets assume the Robot is facing the human player and we want to score in the bucket */

    /** Start Pose of our robot */
    private final Pose startPose = new Pose(135.5, 84, Math.toRadians(90));

    /** Scoring Pose of our robot. It is facing the submersible at a -45 degree (315 degree) angle. */
    private final Pose specimenPose1 = new Pose(75, 120, Math.toRadians(90));      //score pose1

    private final Pose specimenControlPose1 = new Pose(100, 107, Math.toRadians(90));

    /** Lowest (First) Sample from the Spike Mark */
    private final Pose firstPush = new Pose(130, 120, Math.toRadians(90));      //push

    /** Middle (Second) Sample from the Spike Mark */
    private final Pose specimenPose2 = new Pose(85, 130, Math.toRadians(90));       //score pose2

    private final Pose specimenControlPose2 = new Pose(70, 120, Math.toRadians(90));

    /** Highest (Third) Sample from the Spike Mark */
    private final Pose secondPush = new Pose(130, 131, Math.toRadians(90));

    /** Park Pose for our robot, after we do all of the scoring. */

    private final Pose specimenPose3 = new Pose(85, 135.8, Math.toRadians(90));       //score pose2

    private final Pose specimenControlPose3 = new Pose(65, 130, Math.toRadians(90));

    /** Highest (Third) Sample from the Spike Mark */
    private final Pose thirdPush = new Pose(130, 135.8, Math.toRadians(90));


    /* These are our Paths and PathChains that we will define in buildPaths() */
    private Path scorePreload, lineup;
    private PathChain push1, push2, push3;

    /** Build the paths for the auto (adds, for example, constant/linear headings while doing paths)
     * It is necessary to do this so that all the paths are built before the auto starts. **/
    public void buildPaths() {

        /* There are two major types of paths components: BezierCurves and BezierLines.
         *    * BezierCurves are curved, and require >= 3 points. There are the start and end points, and the control points.
         *    - Control points manipulate the curve between the start and end points.
         *    - A good visualizer for this is [this](https://pedro-path-generator.vercel.app/).
         *    * BezierLines are straight, and require 2 points. There are the start and end points.
         * Paths have can have heading interpolation: Constant, Linear, or Tangential
         *    * Linear heading interpolation:
         *    - Pedro will slowly change the heading of the robot from the startHeading to the endHeading over the course of the entire path.
         *    * Consta3nt Heading Interpolation:
         *    - Pedro will maintain one heading throughout the entire path.
         *    * Tangential Heading Interpolation:
         *    - Pedro will follows the angle of the path such that the robot is always driving forward when it follows the path.
         * PathChains hold Path(s) within it and are able to hold their end point, meaning that they will holdPoint until another path is followed.
         * Here is a explanation of the difference between Paths and PathChains <https://pedropathing.com/commonissues/pathtopathchain.html> */


        //se muta de la poz de start spre primul specimen
        lineup = new Path(new BezierCurve(new Point(startPose), /* Control Point */ new Point(specimenControlPose1), new Point(specimenPose1)));
        lineup.setLinearHeadingInterpolation(startPose.getHeading(), specimenPose1.getHeading());

        /* Here is an example for Constant Interpolation
        scorePreload.setConstantInterpolation(startPose.getHeading()); */

        //impinge primul spcimen spre human player
        push1 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(specimenPose1), new Point(firstPush)))
                .setLinearHeadingInterpolation(specimenPose1.getHeading(), firstPush.getHeading())
                .build();

        // se intoarce de la humam player spre specimenul 2
        lineup = new Path(new BezierCurve(new Point(firstPush), /* Control Point */ new Point(specimenControlPose2), new Point(specimenPose2)));
        lineup.setLinearHeadingInterpolation(firstPush.getHeading(), specimenPose2.getHeading());

        // impinge specimenul 2 spre human player
        push2 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(specimenPose2), new Point(secondPush)))
                .setLinearHeadingInterpolation(specimenPose2.getHeading(), secondPush.getHeading())
                .build();

        // se intoarce spre specimenul 3
        lineup = new Path(new BezierCurve(new Point(secondPush), /* Control Point */ new Point(specimenControlPose3), new Point(specimenPose3)));
        lineup.setLinearHeadingInterpolation(secondPush.getHeading(), specimenPose3.getHeading());

        // duce ultimul specimen spre human player
        push2 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(specimenPose3), new Point(thirdPush)))
                .setLinearHeadingInterpolation(specimenPose3.getHeading(), thirdPush.getHeading())
                .build();
        /* This is our grabPickup2 PathChain. We are using a single path with a BezierLine, which is a straight line. */

    }

    /** This switch is called continuously and runs the pathing, at certain points, it triggers the action state.
     * Everytime the switch changes case, it will reset the timer. (This is because of the setPathState() method)
     * The followPath() function sets the follower to run the specific path, but does NOT wait for it to finish before moving on. */
    /*public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                follower.followPath(scorePreload);
                setPathState(1);
                break;
            case 1:

                *//* You could check for
                - Follower State: "if(!follower.isBusy() {}" (Though, I don't recommend this because it might not return due to holdEnd
                - Time: "if(pathTimer.getElapsedTimeSeconds() > 1) {}"
                - Robot Position: "if(follower.getPose().getX() > 36) {}"
                *//*

                *//* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position *//*
                if(follower.getPose().getX() > (scorePose.getX() - 1) && follower.getPose().getY() > (scorePose.getY() - 1)) {
                    *//* Score Preload *//*
                    claw.scoringClaw();
                    claw.openClaw();
                    *//* Since this is a pathChain, we can have Pedro hold the end point while we are grabbing the sample *//*
                    follower.followPath(grabPickup1,true);
                    setPathState(2);
                }
                break;
            case 2:
                *//* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the pickup1Pose's position *//*
                if(follower.getPose().getX() > (pickup1Pose.getX() - 1) && follower.getPose().getY() > (pickup1Pose.getY() - 1)) {
                    *//* Grab Sample *//*
                    claw.groundClaw();
                    claw.closeClaw();
                    *//* Since this is a pathChain, we can have Pedro hold the end point while we are scoring the sample *//*
                    follower.followPath(scorePickup1,true);
                    setPathState(3);
                }
                break;
            case 3:
                *//* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position *//*
                if(follower.getPose().getX() > (scorePose.getX() - 1) && follower.getPose().getY() > (scorePose.getY() - 1)) {
                    *//* Score Sample *//*
                    claw.scoringClaw();
                    claw.openClaw();
                    *//* Since this is a pathChain, we can have Pedro hold the end point while we are grabbing the sample *//*
                    follower.followPath(grabPickup2,true);
                    setPathState(4);
                }
                break;
            case 4:
                *//* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the pickup2Pose's position *//*
                if(follower.getPose().getX() > (pickup2Pose.getX() - 1) && follower.getPose().getY() > (pickup2Pose.getY() - 1)) {
                    *//* Grab Sample *//*
                    claw.groundClaw();
                    claw.closeClaw();
                    *//* Since this is a pathChain, we can have Pedro hold the end point while we are scoring the sample *//*
                    follower.followPath(scorePickup2,true);
                    setPathState(5);
                }
                break;
            case 5:
                *//* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position *//*
                if(follower.getPose().getX() > (scorePose.getX() - 1) && follower.getPose().getY() > (scorePose.getY() - 1)) {
                    *//* Score Sample *//*
                    claw.scoringClaw();
                    claw.openClaw();
                    *//* Since this is a pathChain, we can have Pedro hold the end point while we are grabbing the sample *//*
                    follower.followPath(grabPickup3,true);
                    setPathState(6);
                }
                break;
            case 6:
                *//* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the pickup3Pose's position *//*
                if(follower.getPose().getX() > (pickup3Pose.getX() - 1) && follower.getPose().getY() > (pickup3Pose.getY() - 1)) {
                    *//* Grab Sample *//*
                    claw.groundClaw();
                    claw.closeClaw();
                    *//* Since this is a pathChain, we can have Pedro hold the end point while we are scoring the sample *//*
                    follower.followPath(scorePickup3, true);
                    setPathState(7);
                }
                break;
            case 7:
                *//* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position *//*
                if(follower.getPose().getX() > (scorePose.getX() - 1) && follower.getPose().getY() > (scorePose.getY() - 1)) {
                    *//* Score Sample *//*
                    claw.scoringClaw();
                    claw.openClaw();
                    *//* Since this is a pathChain, we can have Pedro hold the end point while we are parked *//*
                    follower.followPath(park,true);
                    setPathState(8);
                }
                break;
            case 8:
                *//* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position *//*
                if(follower.getPose().getX() > (parkPose.getX() - 1) && follower.getPose().getY() > (parkPose.getY() - 1)) {
                    *//* Put the claw in position to get a level 1 ascent *//*
                    claw.startClaw();
                    claw.closeClaw();

                    *//* Set the state to a Case we won't use or define, so it just stops running an new paths *//*
                    setPathState(-1);
                }
                break;
        }
    }*/

    /** These change the states of the paths and actions
     * It will also reset the timers of the individual switches **/
    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }

    /** This is the main loop of the OpMode, it will run repeatedly after clicking "Play". **/
    @Override
    public void loop() {

        // These loop the movements of the robot
        follower.update();
//        autonomousPathUpdate();

        // Feedback to Driver Hub
        telemetry.addData("path state", pathState);
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.update();
    }

    /** This method is called once at the init of the OpMode. **/
    @Override
    public void init() {
        pathTimer = new Timer();
        opmodeTimer = new Timer();

        opmodeTimer.resetTimer();

        follower = new Follower(hardwareMap);
        follower.setStartingPose(startPose);

        buildPaths();

//        claw = new ClawSubsystem(hardwareMap);
//
//        // Set the claw to positions for init
//        claw.closeClaw();
//        claw.startClaw();
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
}