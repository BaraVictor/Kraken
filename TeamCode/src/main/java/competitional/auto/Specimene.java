package competitional.auto;

import com.pedropathing.follower.Follower;
import com.pedropathing.util.Constants;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import  com.qualcomm.robotcore.eventloop.opmode.OpMode;

import com.pedropathing.follower.*;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierCurve;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.Path;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;
import com.pedropathing.util.Timer;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import pedroPathing.constants.FConstants;
import pedroPathing.constants.LConstants;

/**
 * This is an example auto that showcases movement and control of two servos autonomously.
 * There are examples of different ways to build paths.
 * A path progression method has been created and can advance based on time, position, or other factors.
 *
 * @author Baron Henderson - 20077 The Indubitables
 * @version 2.0, 11/28/2024
 */

@Autonomous(name = "Specimene", group = "Simple")
public class Specimene extends OpMode {

    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;

    /** This is the variable where we store the state of our auto.
     * It is used by the pathUpdate method. */
    private int pathState;
    private final Pose startPose = new Pose(135.5, 89, Math.toRadians(180));

    private final Pose samplePose1 = new Pose(90, 120, Math.toRadians(180));      //score pose1

    private final Pose sampleControlPose1 = new Pose(100, 107, Math.toRadians(180));

    private final Pose firstPush = new Pose(130, 120, Math.toRadians(180));      //push

    private final Pose samplePose2 = new Pose(90, 130, Math.toRadians(180));       //score pose2

    private final Pose sampleControlPose2 = new Pose(70, 120, Math.toRadians(180));

    private final Pose secondPush = new Pose(130, 131, Math.toRadians(180));


    private final Pose samplePose3 = new Pose(90, 140, Math.toRadians(180));       //score pose2

    private final Pose sampleControlPose3 = new Pose(65, 125, Math.toRadians(180));

    private final Pose thirdPush = new Pose(130, 137, Math.toRadians(180));


    /* These are our Paths and PathChains that we will define in buildPaths() */
    private Path scorePreload, lineup1, lineup2, lineup3;
    private PathChain push1, push2, push3;

    /** Build the paths for the auto (adds, for example, constant/linear headings while doing paths)
     * It is necessary to do this so that all the paths are built before the auto starts. **/
    public void buildPaths() {
        //se muta de la poz de start spre primul sample
        lineup1 = new Path(new BezierLine(new Point(startPose), /* Control Point */  new Point(samplePose1)));
        lineup1.setLinearHeadingInterpolation(startPose.getHeading(), samplePose1.getHeading());

        //impinge primul spcimen spre human player
        push1 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(samplePose1), new Point(firstPush)))
                .setLinearHeadingInterpolation(samplePose1.getHeading(), firstPush.getHeading())
                .build();

        // se intoarce de la humam player spre sampleul 2
        lineup2 = new Path(new BezierLine(new Point(firstPush), new Point(samplePose2)));
        lineup2.setLinearHeadingInterpolation(firstPush.getHeading(), samplePose2.getHeading());

        // impinge sampleul 2 spre human player
        push2 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(samplePose2), new Point(secondPush)))
                .setLinearHeadingInterpolation(samplePose2.getHeading(), secondPush.getHeading())
                .build();

        // se intoarce spre sampleul 3
        lineup3 = new Path(new BezierLine(new Point(secondPush), new Point(samplePose3)));
        lineup3.setLinearHeadingInterpolation(secondPush.getHeading(), samplePose3.getHeading());

        // duce ultimul sample spre human player
        push3 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(samplePose3), new Point(thirdPush)))
                .setLinearHeadingInterpolation(samplePose3.getHeading(), thirdPush.getHeading())
                .build();


    }

    /** This switch is called continuously and runs the pathing, at certain points, it triggers the action state.
     * Everytime the switch changes case, it will reset the timer. (This is because of the setPathState() method)
     * The followPath() function sets the follower to run the specific path, but does NOT wait for it to finish before moving on. */
    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                follower.followPath(lineup1);
                setPathState(1);
                break;

            case 1:
                if(!follower.isBusy()){
                    follower.followPath(push1,true);
                    setPathState(2);

                }
                break;

            case 2:
                if(!follower.isBusy()){
                    follower.followPath(lineup2,true);
                    setPathState(3);}
                break;

            case 3:
                if(!follower.isBusy()){
                  follower.followPath(push2,true);
                setPathState(4);}
                break;

            case 4:
                if(!follower.isBusy()){
                   follower.followPath(lineup3,true);
                setPathState(5);}
                break;
            case 5:
                if(!follower.isBusy()){
                  follower.followPath(push3,true);
                setPathState(-1);}
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

        // These loop the movements of the robot
        follower.update();
        autonomousPathUpdate();

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

        Constants.setConstants(FConstants.class, LConstants.class);
        follower = new Follower(hardwareMap);
        follower.setStartingPose(startPose);
        follower.setMaxPower(0.8);
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
}