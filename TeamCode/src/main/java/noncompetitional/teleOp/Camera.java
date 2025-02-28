package noncompetitional.teleOp;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import cvPipelines.ColorRangeTester;
import cvPipelines.RectDrawer;
import cvProcessors.SampleOrientationProcessor;

@TeleOp(name = "Camera")
public class Camera extends LinearOpMode {
    OpenCvCamera camera;
    @Override
    public void runOpMode() throws InterruptedException {
        int cameraMonitorViewId = hardwareMap.appContext
                .getResources().getIdentifier("cameraMonitorViewId",
                        "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class,"camera"), cameraMonitorViewId);
        RectDrawer detector = new RectDrawer(telemetry, RectDrawer.SampleColor.YELLOW);
        camera.setPipeline(detector);
        camera.openCameraDevice();
        camera.startStreaming(640, 480, OpenCvCameraRotation.UPRIGHT);
        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());
        FtcDashboard.getInstance().startCameraStream(camera, 30);
        waitForStart();
    }
}
