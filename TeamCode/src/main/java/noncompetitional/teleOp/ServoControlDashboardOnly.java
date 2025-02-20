package noncompetitional.teleOp;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;

import constants.OuttakeConstants;
import constants.ServoConstants;

@Config
@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "Servo Control with Dashboard Only", group = "Test")
public class ServoControlDashboardOnly extends LinearOpMode {
    private FtcDashboard dashboard;

    // Declarație pentru servo-uri
    private Servo outtakeElbowLeftServo, outtakeClawServo, outtakeWristRotServo, outtakeWristYServo, outtakeElbowRightServo;
    private Servo intakeElbowRightServo, intakeElbowLeftServo, intakeWristServo, intakeWristRightServo, intakeWristLeftServo, intakeClawServo, intakeWristRotServo;

    // Variabile publice pentru FTC Dashboard (una pentru fiecare servo)
    public static double outtakeElbowLeftAndRightServoPos;
    public static double outtakeClawServoPos;
    public static double outtakeWristRotServoPos;
    public static double outtakeWristYServoPos;

    public static double intakeElbowLeftAndRightServoPos;
    public static double intakeWristServoPos;
    public static double intakeWristLeftAndRightServoPos;
    public static double intakeClawServoPos;
    public static double intakeWristRotServoPos;
    @Override
    public void runOpMode() {
        // Inițializarea servo-urilor din hardware map
        outtakeElbowLeftServo = hardwareMap.get(Servo.class, "outtakeElbowLeftServo");
        outtakeClawServo = hardwareMap.get(Servo.class, "outtakeClawServo");
        outtakeWristRotServo = hardwareMap.get(Servo.class, "outtakeWristRotServo");
        outtakeWristYServo = hardwareMap.get(Servo.class, "outtakeWristYServo");
        outtakeElbowRightServo = hardwareMap.get(Servo.class, "outtakeElbowRightServo");
        intakeElbowRightServo = hardwareMap.get(Servo.class, "intakeElbowRightServo");
        intakeElbowLeftServo = hardwareMap.get(Servo.class, "intakeElbowLeftServo");
        intakeWristServo = hardwareMap.get(Servo.class, "intakeWristServo");
        intakeWristRightServo = hardwareMap.get(Servo.class, "intakeWristRightServo");
        intakeWristLeftServo = hardwareMap.get(Servo.class, "intakeWristLeftServo");
        intakeClawServo = hardwareMap.get(Servo.class, "intakeClawServo");
        intakeWristRotServo = hardwareMap.get(Servo.class, "intakeWristRotServo");

        // Citirea pozițiilor curente ale servo-urilor
        outtakeElbowLeftAndRightServoPos = ServoConstants.OUTTAKE_ELBOW_LEFT_PICKUP_POSITION;
        outtakeClawServoPos = ServoConstants.OUTTAKE_CLAW_CLOSED_POSITION;
        outtakeWristRotServoPos = ServoConstants.OUTTAKE_WRIST_ROT_180_DEGREES;
        outtakeWristYServoPos = ServoConstants.OUTTAKE_WRIST_Y_TRANSFER_POSITION;
        intakeElbowLeftAndRightServoPos = ServoConstants.INTAKE_ELBOW_LEFT_RETRACTED_POSITION;
        intakeWristServoPos = ServoConstants.INTAKE_WRIST_UP;
        intakeWristLeftAndRightServoPos = ServoConstants.INTAKE_WRIST_LEFT_UP_POSITION;
        intakeClawServoPos = ServoConstants.INTAKE_CLAW_OPEN_POSITION;
        intakeWristRotServoPos = ServoConstants.INTAKE_WRIST_ROT_0_DEGREES;

        telemetry.addData("Initial Positions Set", "All servos locked to current position.");
        telemetry.update();

        dashboard = FtcDashboard.getInstance();
        dashboard.setTelemetryTransmissionInterval(25);
        telemetry = dashboard.getTelemetry();

        waitForStart();

        while (opModeIsActive()) {
            // Actualizare poziții servo-uri doar dacă sunt modificate în Dashboard
            if (outtakeElbowLeftServo.getPosition() != outtakeElbowLeftAndRightServoPos) {
                outtakeElbowLeftServo.setPosition(outtakeElbowLeftAndRightServoPos);
                outtakeElbowRightServo.setPosition(outtakeElbowLeftAndRightServoPos);
            }
            if (outtakeClawServo.getPosition() != outtakeClawServoPos) {
                outtakeClawServo.setPosition(outtakeClawServoPos);
            }
            if (outtakeWristRotServo.getPosition() != outtakeWristRotServoPos) {
                outtakeWristRotServo.setPosition(outtakeWristRotServoPos);
            }
            if (outtakeWristYServo.getPosition() != outtakeWristYServoPos) {
                outtakeWristYServo.setPosition(outtakeWristYServoPos);
            }

            if (intakeElbowRightServo.getPosition() != intakeElbowLeftAndRightServoPos) {
                intakeElbowRightServo.setPosition(intakeElbowLeftAndRightServoPos);
                intakeElbowLeftServo.setPosition(intakeElbowLeftAndRightServoPos);
            }
            if (intakeWristServo.getPosition() != intakeWristServoPos) {
                intakeWristServo.setPosition(intakeWristServoPos);
            }
            if (intakeWristRightServo.getPosition() != intakeWristLeftAndRightServoPos) {
                intakeWristRightServo.setPosition(intakeWristLeftAndRightServoPos);
                intakeWristLeftServo.setPosition(intakeWristLeftAndRightServoPos);
            }
            if (intakeClawServo.getPosition() != intakeClawServoPos) {
                intakeClawServo.setPosition(intakeClawServoPos);
            }
            if (intakeWristRotServo.getPosition() != intakeWristRotServoPos) {
                intakeWristRotServo.setPosition(intakeWristRotServoPos);
            }

            // Afișare valori actuale pe dashboard
            telemetry.addData("Outtake Claw Servo", outtakeClawServoPos);
            telemetry.addData("Outtake Wrist Rot Servo", outtakeWristRotServoPos);
            telemetry.addData("Outtake Wrist Y Servo", outtakeWristYServoPos);
            telemetry.addData("Intake Wrist Servo", intakeWristServoPos);
            telemetry.addData("Intake Claw Servo", intakeClawServoPos);
            telemetry.addData("Intake Wrist Rot Servo", intakeWristRotServoPos);

            telemetry.update();
        }
    }
}
