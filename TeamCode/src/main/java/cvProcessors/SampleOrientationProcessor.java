package cvProcessors;

import android.graphics.Canvas;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import cvPipelines.RectDrawer;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfDouble;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.core.RotatedRect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.HashSet;
import java.util.List;
import java.util.Set;

@Config
public class SampleOrientationProcessor extends OpenCvPipeline {

    public enum SampleColor {
        YELLOW,
        BLUE,
        RED
    }

    private Mat frame;
    private Telemetry telemetry;

    public static Scalar lowerYellow = new Scalar(19.0, 102.0, 130.1); // hsv
    public static Scalar upperYellow = new Scalar(30.0, 255.0, 255.0); // hsv
    public static Scalar lowerBlue = new Scalar(90.0, 90.0, 90.0); // hsv
    public static Scalar upperBlue = new Scalar(120.0, 255.0, 255.0); // hsv
    public static Scalar lowerRedH = new Scalar(10.0, 0.0, 0.0); // hsv
    public static Scalar upperRedH = new Scalar(170.0, 255.0, 255.0); // hsv
    public static Scalar lowerRedSV = new Scalar(0.0, 130.0, 100.0); // hsv
    public static Scalar upperRedSV = new Scalar(255.0, 255.0, 255.0); // hsv

    private double sampleAngle = 0;
    private double averageBrightness = 0;

    public static RectDrawer.SampleColor colorType = RectDrawer.SampleColor.YELLOW;

    private ArrayList<RotatedRect> rects;

    public SampleOrientationProcessor(Telemetry telemetry) {
        this.telemetry = telemetry;
    }

    @Override
    public Mat processFrame(Mat input) {
        frame = input;

        // Convertim imaginea în HSV și grayscale
        Mat hsv = new Mat();
        Imgproc.cvtColor(input, hsv, Imgproc.COLOR_RGB2HSV);
        Mat gray = new Mat();
        Imgproc.cvtColor(input, gray, Imgproc.COLOR_RGB2GRAY);

        // Aplicăm threshold pentru culoare
        Mat inRange = new Mat();
        if (colorType.equals(RectDrawer.SampleColor.BLUE)) {
            Core.inRange(hsv, lowerBlue, upperBlue, inRange);
        } else if (colorType.equals(RectDrawer.SampleColor.RED)) {
            Mat inHRange = new Mat();
            Mat inSVRange = new Mat();
            Core.inRange(hsv, lowerRedH, upperRedH, inHRange);
            Core.bitwise_not(inHRange, inHRange);
            Core.inRange(hsv, lowerRedSV, upperRedSV, inSVRange);
            Core.bitwise_and(inHRange, inSVRange, inRange);
        } else {
            Core.inRange(hsv, lowerYellow, upperYellow, inRange);
        }

        // Opțiuni de morfologie (comentate, pot fi activate dacă e necesar)
        Mat kernel = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(25, 25));
        Mat kernel2 = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(10, 10));
        // Imgproc.erode(inRange, inRange, kernel);
        // Imgproc.dilate(inRange, inRange, kernel2);

        // Găsim contururile
        List<MatOfPoint> unfilteredContours = new ArrayList<>();
        Mat hierarchy = new Mat();
        Imgproc.findContours(inRange, unfilteredContours, hierarchy, Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);

        // Filtrăm contururile pe baza ariei și obținem dreptunghiurile rotite
        int minArea = 2500;
        ArrayList<RotatedRect> rotatedRects = new ArrayList<>();
        List<MatOfPoint> filteredContours = new ArrayList<>();
        for (MatOfPoint contour : unfilteredContours) {
            RotatedRect minAreaRect = Imgproc.minAreaRect(new MatOfPoint2f(contour.toArray()));
            double area = minAreaRect.size.area();
            if (area > minArea) {
                filteredContours.add(contour);
                rotatedRects.add(minAreaRect);
            }
        }
        Imgproc.drawContours(frame, filteredContours, -1, new Scalar(0, 255, 0), 2);

        // Grupăm dreptunghiurile rotite suprapuse
        double overlapThreshold = 0.2; // procent din dreptunghiul mai mic acoperit
        Set<Integer> toSkip = new HashSet<>();
        ArrayList<ArrayList<Double[]>> overlapGroups = new ArrayList<>();
        for (int i = 0; i < rotatedRects.size(); i++) {
            if (toSkip.contains(i)) continue;
            toSkip.add(i);
            ArrayList<Double[]> overlapGroup = new ArrayList<>();
            double iArea = rotatedRects.get(i).size.area();
            overlapGroup.add(new Double[]{(double) i, iArea});
            for (int j = i + 1; j < rotatedRects.size(); j++) {
                if (toSkip.contains(j)) continue;
                double jArea = rotatedRects.get(j).size.area();
                for (Double[] rect : overlapGroup) {
                    double overlapArea = getIntersectionArea(rotatedRects.get(rect[0].intValue()), rotatedRects.get(j));
                    if (overlapArea / Math.min(rect[1], jArea) >= overlapThreshold) {
                        overlapGroup.add(new Double[]{(double) j, jArea});
                        toSkip.add(j);
                        break;
                    }
                }
            }
            overlapGroups.add(overlapGroup);
        }

        // (Optional) Afișăm grupurile de suprapunere pe telemetry
        ArrayList<ArrayList<Double>> overlapGroups2 = new ArrayList<>();
        for (ArrayList<Double[]> overlapGroup : overlapGroups) {
            ArrayList<Double> groupIndices = new ArrayList<>();
            for (Double[] index : overlapGroup) {
                groupIndices.add(index[0]);
            }
            overlapGroups2.add(groupIndices);
        }
        telemetry.addData("overlapGroups", overlapGroups2);

        // Se selectează dreptunghiul cu cea mai mare arie din fiecare grup
        ArrayList<RotatedRect> filteredRects = new ArrayList<>();
        for (ArrayList<Double[]> overlapGroup : overlapGroups) {
            int maxIndex = overlapGroup.get(0)[0].intValue();
            double maxArea = overlapGroup.get(0)[1];
            for (Double[] rect : overlapGroup) {
                if (rect[1] > maxArea) {
                    maxArea = rect[1];
                    maxIndex = rect[0].intValue();
                }
            }
            filteredRects.add(rotatedRects.get(maxIndex));
        }

        telemetry.addData("filteredRects.size()", filteredRects.size());

        // Desenăm dreptunghiurile inițiale (albastru)
        for (RotatedRect rotatedRect : rotatedRects) {
            Point[] vertices = new Point[4];
            rotatedRect.points(vertices);
            for (int i = 0; i < 4; i++) {
                Imgproc.line(frame, vertices[i], vertices[(i + 1) % 4], new Scalar(0, 0, 255), 2);
            }
        }

        // Desenăm dreptunghiurile filtrate (verde) și centrul acestora (galben)
        for (RotatedRect rotatedRect : filteredRects) {
            Point[] vertices = new Point[4];
            rotatedRect.points(vertices);
            for (int i = 0; i < 4; i++) {
                Imgproc.line(frame, vertices[i], vertices[(i + 1) % 4], new Scalar(0, 255, 0), 2);
            }
            Imgproc.circle(frame, rotatedRect.center, 5, new Scalar(255, 255, 0));
        }
        rects = new ArrayList<>(filteredRects);

        // Calculăm unghiul de procesare pe baza primului dreptunghi filtrat
        if (!filteredRects.isEmpty()) {
            RotatedRect firstRect = filteredRects.get(0);
            telemetry.addData("width ", firstRect.size.width);
            telemetry.addData("height ", firstRect.size.height);
            telemetry.addData("angle ", firstRect.angle);
            telemetry.addData("center ", firstRect.center);
            double procAngle = firstRect.angle;
            if (firstRect.size.width > firstRect.size.height)
                procAngle *= -1;
            else
                procAngle = 90 - procAngle;
            telemetry.addData("procAngle ", procAngle);
            sampleAngle = Math.toRadians(procAngle);
        }
        telemetry.addData("sampleAngle", sampleAngle);

        // Calculăm luminozitatea medie a imaginii (pe baza zonei în jurul mediei)
        MatOfDouble muMat = new MatOfDouble();
        MatOfDouble sigmaMat = new MatOfDouble();
        Core.meanStdDev(gray, muMat, sigmaMat);
        double mu = muMat.get(0, 0)[0];
        double sigma = sigmaMat.get(0, 0)[0];
        double k = 1;
        Scalar lowerBound = new Scalar(mu - k * sigma);
        Scalar upperBound = new Scalar(mu + k * sigma);
        Mat mask = new Mat();
        Core.inRange(gray, lowerBound, upperBound, mask);
        Scalar maskedMean = Core.mean(gray, mask);
        double averageInRange = maskedMean.val[0];
        averageBrightness = averageInRange;
        telemetry.addData("averageBrightness", averageBrightness);

        telemetry.update();

        return frame;
    }

    public double getSampleAngle() {
        return sampleAngle;
    }

    public double getAverageBrightness() {
        return averageBrightness;
    }

    private double getIntersectionArea(RotatedRect rect1, RotatedRect rect2) {
        // Obținem colțurile dreptunghiurilor
        Point[] vertices1 = new Point[4];
        rect1.points(vertices1);
        Point[] vertices2 = new Point[4];
        rect2.points(vertices2);
        MatOfPoint2f poly1 = new MatOfPoint2f(vertices1);
        MatOfPoint2f poly2 = new MatOfPoint2f(vertices2);
        MatOfPoint2f intersection = new MatOfPoint2f();
        return Imgproc.intersectConvexConvex(poly1, poly2, intersection, true);
    }

    // Metodă opțională pentru desenare suplimentară pe canvas (nu este parte a OpenCvPipeline)
    public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight, float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext) {
        // Poți implementa desenări suplimentare pe canvas aici, dacă e nevoie.
    }

    // Calculează offset-urile (scalate în inci) pe baza poziției dreptunghiurilor
    public ArrayList<Point> getOffsets() {
        ArrayList<Point> output = new ArrayList<>();
        double height = 3.0; // inci
        double canvasVertical = height * 3.0 / 8.0; // inci
        double canvasHorizontal = height / 2.0;

        if (rects != null) {
            for (RotatedRect i : rects) {
                // Se presupune că centrul real este (320, 480) cu direcția pozitivă spre dreapta și în jos
                output.add(new Point((i.center.x - 320) / 320 * canvasHorizontal,
                        -(i.center.y - 240) / 240 * canvasVertical));
            }
        }
        return output;
    }
}
