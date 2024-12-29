package org.firstinspires.ftc.teamcode.noncompetitional.teleOp;

import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp(name = "Detect Colors (HSV) and Distance", group = "Sensor")
public class DetectColorsAndDistance extends LinearOpMode {

    private ColorSensor colorSensor;    // REV Color Sensor V3
    private DistanceSensor distanceSensor; // Proximity sensor

    @Override
    public void runOpMode() {
        // Initialize the sensor
        colorSensor = hardwareMap.get(ColorSensor.class, "colorSensor");
        distanceSensor = hardwareMap.get(DistanceSensor.class, "colorSensor"); // Distance is part of the same device

        float[] hsvValues = {0F, 0F, 0F}; // HSV array to store converted values

        // Wait for start
        waitForStart();

        while (opModeIsActive()) {
            // Convert RGB to HSV
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
            boolean isYellow = (hue >= 20 && hue <= 50) && (saturation >= 0.5) && (value >= 0.5);
            boolean isBlue = (hue >= 200 && hue <= 260) && (saturation >= 0.5) && (value >= 0.5);
            boolean isRed = ((hue >= 0 && hue <= 20) || (hue >= 340 && hue <= 360)) && (saturation >= 0.5) && (value >= 0.5);

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

            sleep(50);
        }
    }
}
