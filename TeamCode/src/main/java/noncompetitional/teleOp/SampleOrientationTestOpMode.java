package noncompetitional.teleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvInternalCamera;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.opencv.core.Point;
import java.util.ArrayList;
import cvProcessors.SampleOrientationProcessor;

@TeleOp(name = "Sample Orientation Test", group = "TeleOp")
public class SampleOrientationTestOpMode extends LinearOpMode {

    private OpenCvCamera camera;
    private SampleOrientationProcessor processor;

    @Override
    public void runOpMode() {
        // Inițializează processorul de vedere
        processor = new SampleOrientationProcessor(telemetry);

        // Obține ID-ul view-ului pentru monitorizarea camerei
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());

        // Creează instanța camerei folosind OpenCvInternalCamera.CameraDirection
        camera = OpenCvCameraFactory.getInstance().createInternalCamera(
                OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);

        // Setează pipeline-ul – asigură-te că SampleOrientationProcessor extinde OpenCvPipeline!
        camera.setPipeline(processor);

        // Deschide camera asincron
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                // Pornește streaming-ul cu o rezoluție de 640x480
                camera.startStreaming(640, 480, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {
                telemetry.addData("Camera Error", errorCode);
            }
        });

        telemetry.addData("Status", "Aștept startul...");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            // Obține valorile calculate de processor
            double sampleAngle = processor.getSampleAngle();
            double averageBrightness = processor.getAverageBrightness();
            ArrayList<Point> offsets = processor.getOffsets();

            telemetry.addData("Unghi (grade)", Math.toDegrees(sampleAngle));
            telemetry.addData("Luminozitate medie", averageBrightness);
            telemetry.addData("Offset-uri", offsets.toString());
            telemetry.update();

            sleep(100);
        }

        // Oprește streaming-ul camerei la terminarea opmode-ului
        camera.stopStreaming();
    }
}
