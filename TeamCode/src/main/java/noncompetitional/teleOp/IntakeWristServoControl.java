package noncompetitional.teleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;
import org.openftc.easyopencv.OpenCvPipeline;

@TeleOp(name = "ServoControlWithOpenCV")
public class IntakeWristServoControl extends LinearOpMode {

    private OpenCvCamera camera;
    private Servo intakeWristRotServo;
    private static final double INTAKE_WRIST_ROT_90_DEGREES = 0.431; // Poziția 0° (începem cu maximul)
    private static final double INTAKE_WRIST_ROT_0_DEGREES = 1.0; // Poziția 90° (începem cu minimul)
    private double servoPosition = INTAKE_WRIST_ROT_0_DEGREES; // Poziția implicită la 90° (minim)

    // Variabile pentru controlul delay-ului
    private static final long UPDATE_INTERVAL_MS = 100; // Intervalul de actualizare în milisecunde
    private long lastUpdateTime = 0; // Timpul ultimei actualizări
    private double lastServoPosition = INTAKE_WRIST_ROT_0_DEGREES; // Ultima poziție setată a servo-ului

    private boolean isRotated = false; // Flag pentru a evita rotirea continuă

    @Override
    public void runOpMode() {
        // Inițializare cameră OpenCV
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);
        intakeWristRotServo = hardwareMap.get(Servo.class, "intakeWristRotServo");

        ObjectDetectionPipeline pipeline = new ObjectDetectionPipeline();
        camera.setPipeline(pipeline);

        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                camera.startStreaming(640, 480, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {
                telemetry.addData("Camera Error", errorCode);
                telemetry.update();
            }
        });

        waitForStart();

        while (opModeIsActive()) {
            long currentTime = System.currentTimeMillis();

            // Verificăm dacă este timpul pentru o actualizare a servo-ului
            if (currentTime - lastUpdateTime >= UPDATE_INTERVAL_MS && !isRotated) {
                double detectedAngle = pipeline.getDetectedAngle();

                // Rotim unghiul detectat pentru a avea valori între 0° și 90° inversate
                detectedAngle = Math.max(0, Math.min(90, detectedAngle)); // Asigurăm că unghiul este între 0 și 90

                // Asociem unghiurile 0° și 45° cu poziții de servo fixe (inversăm)
                if (detectedAngle <= 22.5) {
                    servoPosition = INTAKE_WRIST_ROT_0_DEGREES; // 90° - Poziția maximă
                } else if (detectedAngle <= 67.5) {
                    servoPosition = INTAKE_WRIST_ROT_90_DEGREES; // 0° - Poziția minimă
                }

                // Verificăm dacă poziția servo-ului s-a schimbat semnificativ față de ultima actualizare
                if (Math.abs(servoPosition - lastServoPosition) > 0.01) {
                    intakeWristRotServo.setPosition(servoPosition);
                    lastServoPosition = servoPosition; // Salvăm noua valoare
                }

                // Actualizăm timpul ultimei actualizări
                lastUpdateTime = currentTime;

                // Verificăm dacă sample-ul a fost detectat
                boolean sampleDetected = pipeline.isSampleDetected();

                // Telemetrie pentru debugging
                telemetry.addData("Detected Angle", detectedAngle);
                telemetry.addData("Servo Position", servoPosition);
                telemetry.addData("Sample Detected", sampleDetected ? "YES" : "NO");
                telemetry.update();
            }

            // Dacă butonul A este apăsat, resetăm flag-ul pentru a detecta un nou unghi
            if (gamepad1.a) {
                isRotated = false;
            }

            sleep(50); // Încetinim loop-ul pentru stabilitate
        }
    }

    static class ObjectDetectionPipeline extends OpenCvPipeline {
        private double detectedAngle = 0;
        private boolean sampleDetected = false;

        // Adăugăm Mat pentru masca de culori
        private Mat matFrame = new Mat();
        private Mat hsvFrame = new Mat();
        private Mat maskRed = new Mat();
        private Mat maskBlue = new Mat();
        private Mat maskYellow = new Mat();
        private Mat combinedMask = new Mat();
        private Mat coloredResult = new Mat();

        // Intervalele de culoare HSV pentru roșu, albastru și galben
        private Scalar lowerRed1 = new Scalar(0, 100, 100);
        private Scalar upperRed1 = new Scalar(10, 255, 255);
        private Scalar lowerRed2 = new Scalar(170, 100, 100);
        private Scalar upperRed2 = new Scalar(180, 255, 255);
        private Scalar lowerBlue = new Scalar(100, 150, 0);
        private Scalar upperBlue = new Scalar(140, 255, 255);
        private Scalar lowerYellow = new Scalar(15, 100, 100);
        private Scalar upperYellow = new Scalar(30, 255, 255);

        @Override
        public Mat processFrame(Mat input) {
            // Conversia imaginii în format HSV
            Imgproc.cvtColor(input, hsvFrame, Imgproc.COLOR_RGB2HSV);

            // Aplicăm intervalele pentru fiecare culoare
            Core.inRange(hsvFrame, lowerRed1, upperRed1, maskRed);
            Core.inRange(hsvFrame, lowerRed2, upperRed2, maskRed);
            Core.inRange(hsvFrame, lowerBlue, upperBlue, maskBlue);
            Core.inRange(hsvFrame, lowerYellow, upperYellow, maskYellow);

            // Combinăm masca pentru toate culorile
            Core.add(maskRed, maskBlue, combinedMask);
            Core.add(combinedMask, maskYellow, combinedMask);

            // Aplicăm masca combinată pe cadrul original
            Core.bitwise_and(input, input, coloredResult, combinedMask);

            // Verificăm dacă a fost detectată o culoare
            sampleDetected = Core.countNonZero(combinedMask) > 500; // Pragul poate fi ajustat

            // Extragem unghiul în funcție de zona detectată
            Mat roi = new Mat(input, new Rect(200, 150, 240, 180)); // Zona de interes
            Mat gray = new Mat();
            Imgproc.cvtColor(roi, gray, Imgproc.COLOR_RGB2GRAY);
            Imgproc.GaussianBlur(gray, gray, new Size(5, 5), 0);
            Mat edges = new Mat();
            Imgproc.Canny(gray, edges, 50, 150);

            // Calculul unghiului
            detectedAngle = calculateAngleFromEdges(edges);

            // Eliberăm resursele
            roi.release();
            gray.release();
            edges.release();

            return input;
        }

        private double calculateAngleFromEdges(Mat edges) {
            // Aplicăm HoughLines pentru a detecta linii
            Mat lines = new Mat();
            Imgproc.HoughLines(edges, lines, 1, Math.PI / 180, 100); // Parametrii pot fi ajustați

            double angle = 0;
            int lineCount = lines.rows();

            // Dacă au fost detectate linii
            if (lineCount > 0) {
                // Calculăm unghiul mediu dintre linii
                double sumAngles = 0;
                for (int i = 0; i < lineCount; i++) {
                    double rho = lines.get(i, 0)[0];
                    double theta = lines.get(i, 0)[1];

                    // Convertim theta din radiani în grade
                    double degreeAngle = Math.toDegrees(theta);
                    sumAngles += degreeAngle;
                }

                angle = sumAngles / lineCount; // Mediam unghiurile linii detectate
            }

            // Eliberăm resursele
            lines.release();

            // Returnăm unghiul calculat
            return angle;
        }

        public double getDetectedAngle() {
            return detectedAngle;
        }

        public boolean isSampleDetected() {
            return sampleDetected;
        }
    }
}