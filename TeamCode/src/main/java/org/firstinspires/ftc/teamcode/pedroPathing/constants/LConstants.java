package org.firstinspires.ftc.teamcode.pedroPathing.constants;

import com.pedropathing.localization.*;
import com.pedropathing.localization.constants.*;

public class LConstants {
    static {
        ThreeWheelConstants.forwardTicksToInches = 0.0022;
        ThreeWheelConstants.strafeTicksToInches = .0021;
        ThreeWheelConstants.turnTicksToInches = -0.002;
        ThreeWheelConstants.leftY = 13.5 * 0.3937;
        ThreeWheelConstants.rightY = -13 * 0.3937;
        ThreeWheelConstants.strafeX = 9 * 0.3937;
        ThreeWheelConstants.rightEncoder_HardwareMapName = "backRightMotor";
        ThreeWheelConstants.leftEncoder_HardwareMapName = "frontRightMotor";
        ThreeWheelConstants.strafeEncoder_HardwareMapName = "frontLeftMotor";
        ThreeWheelConstants.leftEncoderDirection = Encoder.REVERSE;
        ThreeWheelConstants.rightEncoderDirection = Encoder.REVERSE;
        ThreeWheelConstants.strafeEncoderDirection = Encoder.FORWARD;
    }
}




