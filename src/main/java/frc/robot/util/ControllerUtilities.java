package frc.robot.util;

import edu.wpi.first.math.MathUtil;
import frc.robot.Constants.OperatorConstants;

public class ControllerUtilities {
    private static final double m_controllerExponent = Math.pow(
        OperatorConstants.kExpo,
        OperatorConstants.kExpoRatio);

    private static double expo(double input, double exponent) {
        if (input == 0.0) return 0.0;
        double expo = Math.pow(Math.abs(input), exponent);
        return input < 0 ? -expo : expo;
    }

    public static double adjust(double rawInput)
    {
        return expo(MathUtil.applyDeadband(rawInput, OperatorConstants.kDeadband), m_controllerExponent);
    }
}
