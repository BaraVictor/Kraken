package competitional.auto;

import com.acmerobotics.dashboard.FtcDashboard;
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
import com.qualcomm.robotcore.util.ElapsedTime;

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
    private boolean hasTransfered = false;

    private RobotConfig robotConfig;

    private FtcDashboard dashboard;

    private PIDFController pidfControllerUp;
    private PIDFController pidfControllerDown;

    public static double P = 0.0128;
    public static double I = 0;
    public static double D = 0.00005;
    public static double F = 0.01;
    public static double K = 0;
    public static double targetPosition = 0;

    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;
    private int pathState;

    /*
    TO Do:
    Scurtat timere
    Meshup paths-slidere (actiuni)
    Finalizare parcare

    Done:
    Schimbate pozitiile
    Outtake se da peste cap dupa ce ajunge sus
     */


    private final Pose startPose = new Pose(7, 112, Math.toRadians(270));

    private final Pose score = new Pose(21, 129, Math.toRadians(318));
    private final Pose scorePos2 = new Pose(21, 129, Math.toRadians(318));
    private final Pose scorePos3 = new Pose(21, 129, Math.toRadians(318));
    private final Pose preload = new Pose(21, 126, Math.toRadians(318));
    private final Pose sample1 = new Pose(23.5, 126, Math.toRadians(357));
    private final Pose sample2 = new Pose(23.5, 131, Math.toRadians(6.5));
    private final Pose sample3 = new Pose(25.5, 134, Math.toRadians(25));
    private final Pose park = new Pose(75, 97, Math.toRadians(90));
    private final Pose parkFinal = new Pose(75, 93 , Math.toRadians(90));


    private Path scorePreload;
    private PathChain firstPickup, secondPickup, thirdPickup, parkPath, score1, score2, score3, parkPush;

    public void buildPaths() {

        /* This is our grabPickup1 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        scorePreload = new Path((new BezierLine(new Point(startPose), new Point(score))));
        scorePreload.setLinearHeadingInterpolation(startPose.getHeading(), score.getHeading());
//        scorePreload = follower.pathBuilder()
//                .addPath(new BezierLine(new Point(startPose), new Point(score)))
//                .setLinearHeadingInterpolation(startPose.getHeading(),score.getHeading())
//                .build();

        /* This is our scorePickup1 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        firstPickup = follower.pathBuilder()
                .addPath(new BezierLine(new Point(score), new Point(sample1)))
                .setLinearHeadingInterpolation(score.getHeading(), sample1.getHeading())
                .build();

        score1 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(sample1), new Point(score)))
                .setLinearHeadingInterpolation(sample1.getHeading(), score.getHeading())
                .build();

        /* This is our grabPickup2 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        secondPickup = follower.pathBuilder()
                .addPath(new BezierLine(new Point(score), new Point(sample2)))
                .setLinearHeadingInterpolation(score.getHeading(), sample2.getHeading())
                .build();

        score2 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(sample2), new Point(scorePos2)))
                .setLinearHeadingInterpolation(sample2.getHeading(), scorePos2.getHeading())
                .build();

        /* This is our grabPickup2 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        thirdPickup = follower.pathBuilder()
                .addPath(new BezierLine(new Point(scorePos2), new Point(sample3)))
                .setLinearHeadingInterpolation(scorePos2.getHeading(), sample3.getHeading())
                .build();

        score3 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(sample3), new Point(scorePos3)))
                .setLinearHeadingInterpolation(sample3.getHeading(), scorePos3.getHeading())
                .build();

        parkPath = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(scorePos3), new Point(park), new Point(park)))
                .setLinearHeadingInterpolation(scorePos3.getHeading(), park.getHeading())
                .build();

        parkPush = follower.pathBuilder()
                .addPath(new BezierLine(new Point(park), new Point(parkFinal)))
                .setLinearHeadingInterpolation(park.getHeading(), parkFinal.getHeading())
                .build();


        /* This is our scorePickup2 PathChain. We are using a single path with a BezierLine, which is a straight line. */

        /* This is our park path. We are using a BezierCurve with 3 points, which is a curved line that is curved based off of the control point */
    }

    /**
     * This switch is called continuously and runs the pathing, at certain points, it triggers the action state.
     * Everytime the switch changes case, it will reset the timer. (This is because of the setPathState() method)
     * The followPath() function sets the follower to run the specific path, but does NOT wait for it to finish before moving on.
     */
    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                follower.followPath(scorePreload);
                retractTimer.reset();
                revolutTimer.reset();
                setPathState(1);
                break;

            case 1:
                if (follower.getPose().getX() > (score.getX() - 1) && follower.getPose().getY() > (score.getY() - 1)) {
                    /* Score Preload */
                    if (!deposited) {
                        targetPosition = OuttakeConstants.OUTTAKE_MAX_POSITION;
                        robotConfig.outtakeClawServo.setPosition(ServoConstants.OUTTAKE_CLAW_CLOSED_POSITION);
                        robotConfig.outtakeWristRotServo.setPosition(ServoConstants.OUTTAKE_WRIST_ROT_180_DEGREES);
                    }
                    if (robotConfig.upMotor.getCurrentPosition() > targetPosition - 50) {
                        robotConfig.outtakeElbowRightServo.setPosition(ServoConstants.OUTTAKE_ELBOW_RIGHT_PLACE_SAMPLE_POSITION);
                        robotConfig.outtakeElbowLeftServo.setPosition(ServoConstants.OUTTAKE_ELBOW_LEFT_PLACE_SAMPLE_POSITION);
                        robotConfig.outtakeWristYServo.setPosition(ServoConstants.OUTTAKE_WRIST_Y_PLACE_POSITION);
                        if(revolutTimer.seconds()>2.4){
                            robotConfig.outtakeClawServo.setPosition(ServoConstants.OUTTAKE_CLAW_OPEN_POSITION);
                            if (robotConfig.outtakeClawServo.getPosition() == ServoConstants.OUTTAKE_CLAW_OPEN_POSITION) {
                                deposited = true;
                                if(retractTimer.seconds()> 2.8) {
                                    robotConfig.outtakeElbowRightServo.setPosition(ServoConstants.OUTTAKE_ELBOW_RIGHT_PICKUP_POSITION);
                                    robotConfig.outtakeElbowLeftServo.setPosition(ServoConstants.OUTTAKE_ELBOW_LEFT_PICKUP_POSITION);
                                    if (retractTimer.seconds() > 3) {
                                        targetPosition = OuttakeConstants.OUTTAKE_MIN_POSITION;
                                        if (robotConfig.upMotor.getCurrentPosition() < targetPosition + 15) {
                                            follower.followPath(firstPickup, true);
                                            turningTimer.reset();
                                            setPathState(2);
                                        }
                                    }
                                }

                            }
                        }
                    }
                }
                break;
            case 2:
                if (follower.getPose().getX() > (sample1.getX() - 1) && follower.getPose().getY() > (sample1.getY() - 1)) {
                    robotConfig.setOuttakeServoPositions(
                            ServoConstants.OUTTAKE_CLAW_OPEN_POSITION,
                            ServoConstants.OUTTAKE_WRIST_ROT_180_DEGREES,
                            ServoConstants.OUTTAKE_WRIST_Y_TRANSFER_POSITION,
                            ServoConstants.OUTTAKE_ELBOW_RIGHT_PICKUP_POSITION,
                            ServoConstants.OUTTAKE_ELBOW_LEFT_PICKUP_POSITION
                    );
                    hasTurned = true;
                }
                if (!hovering && Math.toDegrees(follower.getPose().getHeading()) > 357 && hasTurned && turningTimer.seconds() > 1) {
                    robotConfig.setIntakeServoPositions(
                            ServoConstants.INTAKE_ELBOW_RIGHT_EXTENDED_POSITION,
                            ServoConstants.INTAKE_ELBOW_LEFT_EXTENDED_POSITION,
                            ServoConstants.INTAKE_WRIST_DOWN,
                            ServoConstants.INTAKE_WRIST_RIGHT_HOVER_POSITION,
                            ServoConstants.INTAKE_WRIST_LEFT_HOVER_POSITION,
                            ServoConstants.INTAKE_CLAW_OPEN_POSITION,
                            ServoConstants.INTAKE_WRIST_ROT_0_DEGREES
                    );
                    hoverTimer.reset();
                }
                if (robotConfig.intakeElbowRightServo.getPosition() == ServoConstants.INTAKE_ELBOW_RIGHT_EXTENDED_POSITION) {
                    hovering = true;
                    if (!intaking && hoverTimer.seconds()>0.5) {
                        robotConfig.setIntakeServoPositions(
                                ServoConstants.INTAKE_ELBOW_RIGHT_EXTENDED_POSITION,
                                ServoConstants.INTAKE_ELBOW_LEFT_EXTENDED_POSITION,
                                ServoConstants.INTAKE_WRIST_DOWN,
                                ServoConstants.INTAKE_WRIST_RIGHT_DOWN_POSITION,
                                ServoConstants.INTAKE_WRIST_LEFT_DOWN_POSITION,
                                ServoConstants.INTAKE_CLAW_OPEN_POSITION,
                                ServoConstants.INTAKE_WRIST_ROT_0_DEGREES);
                        intaking = true;
                    } else {
                        if (hoverTimer.seconds() > 1.2) {
                            robotConfig.setIntakeServoPositions(
                                    ServoConstants.INTAKE_ELBOW_RIGHT_EXTENDED_POSITION,
                                    ServoConstants.INTAKE_ELBOW_LEFT_EXTENDED_POSITION,
                                    ServoConstants.INTAKE_WRIST_DOWN,
                                    ServoConstants.INTAKE_WRIST_RIGHT_DOWN_POSITION,
                                    ServoConstants.INTAKE_WRIST_LEFT_DOWN_POSITION,
                                    ServoConstants.INTAKE_CLAW_CLOSED_POSITION,
                                    ServoConstants.INTAKE_WRIST_ROT_0_DEGREES
                            );
                            if (robotConfig.intakeClawServo.getPosition() > ServoConstants.INTAKE_CLAW_CLOSED_POSITION - 0.1) {
                                follower.followPath(score1, true);
                                deposited = false;
                                revolutTimer.reset();
                                retractTimer.reset();
                                setPathState(3);
                            }
                        }
                    }
                }
                break;
            case 3:
                if (follower.getPose().getX() > (sample1.getX() - 1) && follower.getPose().getY() > (sample1.getY() - 1)) {
                    if(!transfer) {
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
                        transfer = true;
                        intakeTimer.reset();
                    }
                }
                if(transfer && intakeTimer.seconds() > 0.7  && !hasTransfered) {
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
                    if (intakeTimer.seconds() > .9) {
                        robotConfig.setIntakeServoPositions(
                                ServoConstants.INTAKE_ELBOW_RIGHT_RETRACTED_POSITION,
                                ServoConstants.INTAKE_ELBOW_LEFT_RETRACTED_POSITION,
                                ServoConstants.INTAKE_WRIST_UP,
                                ServoConstants.INTAKE_WRIST_RIGHT_UP_POSITION,
                                ServoConstants.INTAKE_WRIST_LEFT_UP_POSITION,
                                ServoConstants.INTAKE_CLAW_OPEN_POSITION,
                                ServoConstants.INTAKE_WRIST_ROT_0_DEGREES
                        );
                        deposited = false;
                        hasTransfered = true;
                    }
                }
                if (hasTransfered) {
                    if(!deposited) {
                        targetPosition = OuttakeConstants.OUTTAKE_TOP_SAMPLE_BOX;
                        if(!opened) {
                            robotConfig.outtakeClawServo.setPosition(ServoConstants.OUTTAKE_CLAW_CLOSED_POSITION);
                            robotConfig.outtakeWristRotServo.setPosition(ServoConstants.OUTTAKE_WRIST_ROT_180_DEGREES);
                        }
                    }
                    if (robotConfig.upMotor.getCurrentPosition() > targetPosition - 50) {
                        robotConfig.outtakeElbowRightServo.setPosition(ServoConstants.OUTTAKE_ELBOW_RIGHT_PLACE_SAMPLE_POSITION);
                        robotConfig.outtakeElbowLeftServo.setPosition(ServoConstants.OUTTAKE_ELBOW_LEFT_PLACE_SAMPLE_POSITION);
                        robotConfig.outtakeWristYServo.setPosition(ServoConstants.OUTTAKE_WRIST_Y_PLACE_POSITION);
                        if(revolutTimer.seconds()>2.5){
                            robotConfig.outtakeClawServo.setPosition(ServoConstants.OUTTAKE_CLAW_OPEN_POSITION);
                            if (robotConfig.outtakeClawServo.getPosition() == ServoConstants.OUTTAKE_CLAW_OPEN_POSITION) {
                                opened = true;
                                if(retractTimer.seconds()> 2.7) {
                                    robotConfig.outtakeElbowRightServo.setPosition(ServoConstants.OUTTAKE_ELBOW_RIGHT_PICKUP_POSITION);
                                    robotConfig.outtakeElbowLeftServo.setPosition(ServoConstants.OUTTAKE_ELBOW_LEFT_PICKUP_POSITION);
                                    if (retractTimer.seconds() > 2.9) {
                                        targetPosition = OuttakeConstants.OUTTAKE_MIN_POSITION;
                                        deposited = true;
                                    }
                                }
                            }
                        }
                    }


                }
                if(deposited && robotConfig.upMotor.getCurrentPosition() < targetPosition + 21) {
                    hasTurned = false;
                    hovering = false;
                    intaking = false;
                    turningTimer.reset();
                    follower.followPath(secondPickup, true);
                    setPathState(4);
                }
                break;

            //sample 3, to be tested
            case 4:
                if (follower.getPose().getX() > (sample2.getX() - 1) && follower.getPose().getY() > (sample2.getY() - 1)) {
                    robotConfig.setOuttakeServoPositions(
                            ServoConstants.OUTTAKE_CLAW_OPEN_POSITION,
                            ServoConstants.OUTTAKE_WRIST_ROT_180_DEGREES,
                            ServoConstants.OUTTAKE_WRIST_Y_TRANSFER_POSITION,
                            ServoConstants.OUTTAKE_ELBOW_RIGHT_PICKUP_POSITION,
                            ServoConstants.OUTTAKE_ELBOW_LEFT_PICKUP_POSITION
                    );
                    hasTurned = true;
                }
                if (!hovering && Math.toDegrees(follower.getPose().getHeading()) > 0  && hasTurned && turningTimer.seconds() > 1.5) {
                    robotConfig.setIntakeServoPositions(
                            ServoConstants.INTAKE_ELBOW_RIGHT_EXTENDED_POSITION,
                            ServoConstants.INTAKE_ELBOW_LEFT_EXTENDED_POSITION,
                            ServoConstants.INTAKE_WRIST_DOWN,
                            ServoConstants.INTAKE_WRIST_RIGHT_HOVER_POSITION,
                            ServoConstants.INTAKE_WRIST_LEFT_HOVER_POSITION,
                            ServoConstants.INTAKE_CLAW_OPEN_POSITION,
                            ServoConstants.INTAKE_WRIST_ROT_0_DEGREES
                    );
                    hoverTimer.reset();
                }
                if (robotConfig.intakeElbowRightServo.getPosition() == ServoConstants.INTAKE_ELBOW_RIGHT_EXTENDED_POSITION) {
                    hovering = true;
                    if (!intaking && hoverTimer.seconds()>0.5) {
                        robotConfig.setIntakeServoPositions(
                                ServoConstants.INTAKE_ELBOW_RIGHT_EXTENDED_POSITION,
                                ServoConstants.INTAKE_ELBOW_LEFT_EXTENDED_POSITION,
                                ServoConstants.INTAKE_WRIST_DOWN,
                                ServoConstants.INTAKE_WRIST_RIGHT_DOWN_POSITION,
                                ServoConstants.INTAKE_WRIST_LEFT_DOWN_POSITION,
                                ServoConstants.INTAKE_CLAW_OPEN_POSITION,
                                ServoConstants.INTAKE_WRIST_ROT_0_DEGREES);
                        intaking = true;
                    } else {
                        if (hoverTimer.seconds() > 1.2) {
                            robotConfig.setIntakeServoPositions(
                                    ServoConstants.INTAKE_ELBOW_RIGHT_EXTENDED_POSITION,
                                    ServoConstants.INTAKE_ELBOW_LEFT_EXTENDED_POSITION,
                                    ServoConstants.INTAKE_WRIST_DOWN,
                                    ServoConstants.INTAKE_WRIST_RIGHT_DOWN_POSITION,
                                    ServoConstants.INTAKE_WRIST_LEFT_DOWN_POSITION,
                                    ServoConstants.INTAKE_CLAW_CLOSED_POSITION,
                                    ServoConstants.INTAKE_WRIST_ROT_0_DEGREES
                            );
                            if (robotConfig.intakeClawServo.getPosition() > ServoConstants.INTAKE_CLAW_CLOSED_POSITION - 0.1) {
                                follower.followPath(score2, true);
                                deposited = false;
                                transfer = false;
                                hasTransfered = false;
                                closed = false;
                                opened = false;
                                retractTimer.reset();
                                revolutTimer.reset();
                                setPathState(5);
                            }
                        }
                    }
                }
                break;

            case 5:
                if (follower.getPose().getX() > (score.getX() - 1) && follower.getPose().getY() > (score.getY() - 1)) {
                    if(!transfer) {
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
                        transfer = true;
                        intakeTimer.reset();
                    }
                }
                if(transfer && intakeTimer.seconds() > 0.7  && !hasTransfered) {
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
                    if (intakeTimer.seconds() > .9) {
                        robotConfig.setIntakeServoPositions(
                                ServoConstants.INTAKE_ELBOW_RIGHT_RETRACTED_POSITION,
                                ServoConstants.INTAKE_ELBOW_LEFT_RETRACTED_POSITION,
                                ServoConstants.INTAKE_WRIST_UP,
                                ServoConstants.INTAKE_WRIST_RIGHT_UP_POSITION,
                                ServoConstants.INTAKE_WRIST_LEFT_UP_POSITION,
                                ServoConstants.INTAKE_CLAW_OPEN_POSITION,
                                ServoConstants.INTAKE_WRIST_ROT_0_DEGREES
                        );
                        deposited = false;
                        hasTransfered = true;
                    }
                }
                if (hasTransfered) {
                    if(!deposited) {
                        targetPosition = OuttakeConstants.OUTTAKE_TOP_SAMPLE_BOX;
                        if(!opened) {
                            robotConfig.outtakeClawServo.setPosition(ServoConstants.OUTTAKE_CLAW_CLOSED_POSITION);
                            robotConfig.outtakeWristRotServo.setPosition(ServoConstants.OUTTAKE_WRIST_ROT_180_DEGREES);
                        }
                    }
                    if (robotConfig.upMotor.getCurrentPosition() > targetPosition - 50) {
                        robotConfig.outtakeElbowRightServo.setPosition(ServoConstants.OUTTAKE_ELBOW_RIGHT_PLACE_SAMPLE_POSITION);
                        robotConfig.outtakeElbowLeftServo.setPosition(ServoConstants.OUTTAKE_ELBOW_LEFT_PLACE_SAMPLE_POSITION);
                        robotConfig.outtakeWristYServo.setPosition(ServoConstants.OUTTAKE_WRIST_Y_PLACE_POSITION);
                        if(revolutTimer.seconds()>2.5){
                            robotConfig.outtakeClawServo.setPosition(ServoConstants.OUTTAKE_CLAW_OPEN_POSITION);
                            if (robotConfig.outtakeClawServo.getPosition() == ServoConstants.OUTTAKE_CLAW_OPEN_POSITION) {
                                opened = true;
                                if(retractTimer.seconds()>2.7) {
                                    robotConfig.outtakeElbowRightServo.setPosition(ServoConstants.OUTTAKE_ELBOW_RIGHT_PICKUP_POSITION);
                                    robotConfig.outtakeElbowLeftServo.setPosition(ServoConstants.OUTTAKE_ELBOW_LEFT_PICKUP_POSITION);
                                    if (retractTimer.seconds() > 2.9) {
                                        targetPosition = OuttakeConstants.OUTTAKE_MIN_POSITION;
                                        deposited = true;
                                    }
                                }
                            }
                        }
                    }


                }
                if(deposited && robotConfig.upMotor.getCurrentPosition() < targetPosition + 21) {
                    hasTurned = false;
                    hovering = false;
                    intaking = false;
                    turningTimer.reset();
                    follower.followPath(thirdPickup, true);
                    setPathState(6);
                }
                break;

            case 6:
                if (follower.getPose().getX() > (sample3.getX() - 1.5) && follower.getPose().getY() > (sample3.getY() - 1.5)) {
                    robotConfig.setOuttakeServoPositions(
                            ServoConstants.OUTTAKE_CLAW_OPEN_POSITION,
                            ServoConstants.OUTTAKE_WRIST_ROT_180_DEGREES,
                            ServoConstants.OUTTAKE_WRIST_Y_TRANSFER_POSITION,
                            ServoConstants.OUTTAKE_ELBOW_RIGHT_PICKUP_POSITION,
                            ServoConstants.OUTTAKE_ELBOW_LEFT_PICKUP_POSITION
                    );
                    hasTurned = true;
                }
                if (!hovering && Math.toDegrees(follower.getPose().getHeading()) > 10  && hasTurned && turningTimer.seconds() > 2.5 ) {
                    robotConfig.setIntakeServoPositions(
                            ServoConstants.INTAKE_ELBOW_RIGHT_EXTENDED_POSITION,
                            ServoConstants.INTAKE_ELBOW_LEFT_EXTENDED_POSITION,
                            ServoConstants.INTAKE_WRIST_DOWN,
                            ServoConstants.INTAKE_WRIST_RIGHT_HOVER_POSITION,
                            ServoConstants.INTAKE_WRIST_LEFT_HOVER_POSITION,
                            ServoConstants.INTAKE_CLAW_OPEN_POSITION,
                            ServoConstants.INTAKE_WRIST_ROT_0_DEGREES
                    );
                    hoverTimer.reset();
                }
                if (robotConfig.intakeElbowRightServo.getPosition() == ServoConstants.INTAKE_ELBOW_RIGHT_EXTENDED_POSITION) {
                    hovering = true;
                    if (!intaking && hoverTimer.seconds() > 0.5) {
                        robotConfig.setIntakeServoPositions(
                                ServoConstants.INTAKE_ELBOW_RIGHT_EXTENDED_POSITION,
                                ServoConstants.INTAKE_ELBOW_LEFT_EXTENDED_POSITION,
                                ServoConstants.INTAKE_WRIST_DOWN,
                                ServoConstants.INTAKE_WRIST_RIGHT_DOWN_POSITION,
                                ServoConstants.INTAKE_WRIST_LEFT_DOWN_POSITION,
                                ServoConstants.INTAKE_CLAW_OPEN_POSITION,
                                ServoConstants.INTAKE_WRIST_ROT_0_DEGREES);
                        intaking = true;
                    } else {
                        if (hoverTimer.seconds() > 1) {
                            robotConfig.setIntakeServoPositions(
                                    ServoConstants.INTAKE_ELBOW_RIGHT_EXTENDED_POSITION,
                                    ServoConstants.INTAKE_ELBOW_LEFT_EXTENDED_POSITION,
                                    ServoConstants.INTAKE_WRIST_DOWN,
                                    ServoConstants.INTAKE_WRIST_RIGHT_DOWN_POSITION,
                                    ServoConstants.INTAKE_WRIST_LEFT_DOWN_POSITION,
                                    ServoConstants.INTAKE_CLAW_CLOSED_POSITION,
                                    ServoConstants.INTAKE_WRIST_ROT_0_DEGREES
                            );
                            if (robotConfig.intakeClawServo.getPosition() > ServoConstants.INTAKE_CLAW_CLOSED_POSITION - 0.1) {
                                follower.followPath(score2, true);
                                deposited = false;
                                transfer = false;
                                hasTransfered = false;
                                closed = false;
                                opened = false;
                                retractTimer.reset();
                                revolutTimer.reset();
                                setPathState(7);
                            }
                        }
                    }
                }
                break;

            case 7:
                if (follower.getPose().getX() > (scorePos3.getX() - 1) && follower.getPose().getY() > (scorePos3.getY() - 1)) {
                    if(!transfer) {
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
                        transfer = true;
                        intakeTimer.reset();
                    }
                }
                if(transfer && intakeTimer.seconds() > 0.7  && !hasTransfered) {
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
                    if (intakeTimer.seconds() > 1.1) {
                        robotConfig.setIntakeServoPositions(
                                ServoConstants.INTAKE_ELBOW_RIGHT_RETRACTED_POSITION,
                                ServoConstants.INTAKE_ELBOW_LEFT_RETRACTED_POSITION,
                                ServoConstants.INTAKE_WRIST_MID,
                                ServoConstants.INTAKE_WRIST_RIGHT_UP_POSITION,
                                ServoConstants.INTAKE_WRIST_LEFT_UP_POSITION,
                                ServoConstants.INTAKE_CLAW_OPEN_POSITION,
                                ServoConstants.INTAKE_WRIST_ROT_0_DEGREES
                        );
                        deposited = false;
                        hasTransfered = true;
                    }
                }
                if (hasTransfered) {
                    if(!deposited) {
                        targetPosition = OuttakeConstants.OUTTAKE_TOP_SAMPLE_BOX;
                        if(!opened) {
                            robotConfig.outtakeClawServo.setPosition(ServoConstants.OUTTAKE_CLAW_CLOSED_POSITION);
                            robotConfig.outtakeWristRotServo.setPosition(ServoConstants.OUTTAKE_WRIST_ROT_180_DEGREES);
                        }
                    }
                    if (robotConfig.upMotor.getCurrentPosition() > targetPosition - 50) {
                        robotConfig.outtakeElbowRightServo.setPosition(ServoConstants.OUTTAKE_ELBOW_RIGHT_PLACE_SAMPLE_POSITION);
                        robotConfig.outtakeElbowLeftServo.setPosition(ServoConstants.OUTTAKE_ELBOW_LEFT_PLACE_SAMPLE_POSITION);
                        robotConfig.outtakeWristYServo.setPosition(ServoConstants.OUTTAKE_WRIST_Y_PLACE_POSITION);
                        if(revolutTimer.seconds()>2.7){
                            robotConfig.outtakeClawServo.setPosition(ServoConstants.OUTTAKE_CLAW_OPEN_POSITION);
                            if (robotConfig.outtakeClawServo.getPosition() == ServoConstants.OUTTAKE_CLAW_OPEN_POSITION) {
                                opened = true;
                                if(retractTimer.seconds()>3) {
                                    robotConfig.outtakeElbowRightServo.setPosition(ServoConstants.OUTTAKE_ELBOW_RIGHT_PICKUP_POSITION);
                                    robotConfig.outtakeElbowLeftServo.setPosition(ServoConstants.OUTTAKE_ELBOW_LEFT_PICKUP_POSITION);
                                    if (retractTimer.seconds() > 3.2) {
                                        targetPosition = OuttakeConstants.OUTTAKE_MIN_POSITION;
                                        deposited = true;
                                    }
                                }
                            }
                        }
                    }


                }
                if(deposited && robotConfig.upMotor.getCurrentPosition() < targetPosition + 21) {
                    hasTurned = false;
                    hovering = false;
                    intaking = false;
                    turningTimer.reset();
                    follower.followPath(parkPath,true);


                    setPathState(8);
                }
                break;

            case 8:
                if (follower.getPose().getX() > (park.getX() - 1) && follower.getPose().getY() > (park.getY() - 1)){
                    robotConfig.setOuttakeServoPositions(
                            ServoConstants.OUTTAKE_CLAW_CLOSED_POSITION,
                            ServoConstants.OUTTAKE_WRIST_ROT_180_DEGREES,
                            ServoConstants.OUTTAKE_WRIST_Y_TRANSFER_POSITION,
                            ServoConstants.OUTTAKE_ELBOW_RIGHT_PLACE_SAMPLE_POSITION,
                            ServoConstants.OUTTAKE_ELBOW_LEFT_PLACE_SAMPLE_POSITION
                    );
                    follower.followPath(parkPush);
                    setPathState(-1);
                }

                /*
            case 5:
                follower.followPath(thirdPickup,false);
                setPathState(6);
                break;
            case 6:
                follower.followPath(score3,false);
                setPathState(7);
                break;
            case 7:
                follower.followPath(parkPath,false);
                setPathState(-1);
                break;*/

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

        /*dashboard = FtcDashboard.getInstance();
        dashboard.setTelemetryTransmissionInterval(25); // Set telemetry update frequency
        telemetry = dashboard.getTelemetry();*/

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
        /*telemetry.addData("path state", pathState);
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
        telemetry.update();*/
//        dashboard = FtcDashboard.getInstance();
//            dashboard.setTelemetryTransmissionInterval(25);
//            telemetry = dashboard.getTelemetry();

        telemetry.addData("Target Position", targetPosition);
        telemetry.addData("Left Motor Position", robotConfig.upMotor.getCurrentPosition());
        telemetry.addData("Right Motor Position", robotConfig.downMotor.getCurrentPosition());
        telemetry.addData("Left Motor Error", robotConfig.upMotor.getCurrentPosition() - targetPosition);
        telemetry.addData("Right Motor Error", robotConfig.downMotor.getCurrentPosition() - targetPosition);
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
                ServoConstants.INTAKE_WRIST_MID,
                ServoConstants.INTAKE_WRIST_RIGHT_UP_POSITION,
                ServoConstants.INTAKE_WRIST_LEFT_UP_POSITION,
                ServoConstants.INTAKE_CLAW_OPEN_POSITION,
                ServoConstants.INTAKE_WRIST_ROT_0_DEGREES
        );
    }
}