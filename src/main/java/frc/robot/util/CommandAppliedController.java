package frc.robot.util;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.OperatorConstants;

/**
 * AppliedController.
 */
public class CommandAppliedController extends CommandXboxController {

    private double m_controllerExponent; //expo factor for the analog axises

    /**
     * Constructor.
     */
    public CommandAppliedController(int port) {
        super(port);
        this.m_controllerExponent = Math.pow(OperatorConstants.kExpo, OperatorConstants.kExpoRatio);
    }

    public double expo(double input, double exponent) {
        if (input == 0.0) return 0.0;
        double expo = Math.pow(Math.abs(input), exponent);
        return input < 0 ? -expo : expo;
    }

    /**
     * Overrides.
     */

    @Override
    public double getLeftY() {
        return expo(
            MathUtil.applyDeadband(super.getLeftY(), OperatorConstants.kDeadband),
            m_controllerExponent);
    }

    @Override
    public double getRightY() {
        return expo(
            MathUtil.applyDeadband(super.getRightY(), OperatorConstants.kDeadband),
            m_controllerExponent);
    }

    @Override
    public double getLeftX() {
        return expo(
            MathUtil.applyDeadband(super.getLeftX(), OperatorConstants.kDeadband),
            m_controllerExponent);
    }

    @Override
    public double getRightX() {
        return expo(
            MathUtil.applyDeadband(super.getRightX(), OperatorConstants.kDeadband),
            m_controllerExponent);
    }

    @Override
    public double getLeftTriggerAxis() {
        return expo(
            MathUtil.applyDeadband(super.getLeftTriggerAxis(), OperatorConstants.kDeadband),
            m_controllerExponent);
    }

    @Override
    public double getRightTriggerAxis() {
        return expo(
            MathUtil.applyDeadband(super.getRightTriggerAxis(), OperatorConstants.kDeadband),
            m_controllerExponent);
    }
}
