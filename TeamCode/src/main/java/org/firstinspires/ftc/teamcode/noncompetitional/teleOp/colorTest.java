package org.firstinspires.ftc.teamcode.noncompetitional.teleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.SwitchableLight;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.teamcode.configurations.RobotConfig;

@TeleOp(name = "REV Color Sensor", group = "Sensor")
public class colorTest extends LinearOpMode {

    NormalizedColorSensor colorSensor;

    @Override
    public void runOpMode() {
        // Inițializează RobotConfig
        RobotConfig robot = new RobotConfig(hardwareMap);

        // Inițializează senzorul de culoare
        colorSensor = hardwareMap.get(NormalizedColorSensor.class, "sensor_color");

        // Dacă senzorul are lumină comutabilă, activează lumina
        if (colorSensor instanceof SwitchableLight) {
            ((SwitchableLight) colorSensor).enableLight(true);
        }

        telemetry.addLine("Pregătit. Apasă START pentru a începe.");
        telemetry.update();

        // Așteaptă apăsarea butonului START
        waitForStart();

        while (opModeIsActive()) {
            // Obține culorile normalizate
            NormalizedRGBA colors = colorSensor.getNormalizedColors();

            // Conversie la HSV
            final float[] hsvValues = new float[3];
            colors.toColor(); // Asigură conversia
            android.graphics.Color.colorToHSV(colors.toColor(), hsvValues);

            // Afișează valorile RGB normalizate și HSV
            telemetry.addLine()
                    .addData("Red", "%.3f", colors.red)
                    .addData("Green", "%.3f", colors.green)
                    .addData("Blue", "%.3f", colors.blue);
            telemetry.addLine()
                    .addData("Hue", "%.3f", hsvValues[0])
                    .addData("Saturation", "%.3f", hsvValues[1])
                    .addData("Value", "%.3f", hsvValues[2]);
            telemetry.addData("Alpha", "%.3f", colors.alpha);

            // Dacă senzorul are funcție de distanță, afișează distanța
            if (colorSensor instanceof DistanceSensor) {
                telemetry.addData("Distanță (cm)", "%.2f", ((DistanceSensor) colorSensor).getDistance(DistanceUnit.CM));
            }

            // Actualizează telemetria
            telemetry.update();

            // Permite controlul luminii senzorului cu butonul X
            if (gamepad1.x && colorSensor instanceof SwitchableLight) {
                SwitchableLight light = (SwitchableLight) colorSensor;
                light.enableLight(!light.isLightOn());
            }
        }
    }
}