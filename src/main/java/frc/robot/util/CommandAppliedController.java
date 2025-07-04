package frc.robot.util;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.OperatorConstants;

/**
 * AppliedController.
 */
@SuppressWarnings({"all"}) // suppress CheckStyle warnings in this file
public class CommandAppliedController extends CommandXboxController {

    private double m_controllerExponent; //expo factor for the analog axises

    /**
     * Constructor.
     */
    public CommandAppliedController(int port) {
        super(port);
        this.m_controllerExponent = Math.pow(OperatorConstants.kExpo, OperatorConstants.kExpoRatio);
    }

    private double expo(double input, double exponent) {
        if (input == 0.0) return 0.0;
        double expo = Math.pow(Math.abs(input), exponent);
        return input < 0 ? -expo : expo;
    }

    private double adjust(double rawInput)
    {
        double output = MathUtil.applyDeadband(rawInput, OperatorConstants.kDeadband);
        return Math.signum(output) * Math.pow(output, 2);
    }

    /**
     * Overrides.
     */

    @Override
    public double getLeftY() {
        // For simulation, we change the orientation by 90 degrees
        if (RobotBase.isSimulation()) {
            return -1.0 * adjust(super.getLeftX());
        }

        return adjust(super.getLeftY());
    }

    @Override
    public double getRightY() {
        return adjust(super.getRightY());
    }

    @Override
    public double getLeftX() {
        if (RobotBase.isSimulation()) {
            return -1.0 * adjust(-super.getLeftY());
        }

        return adjust(super.getLeftX());
    }

    @Override
    public double getRightX() {
        return adjust(super.getRightX());
    }

    @Override
    public double getLeftTriggerAxis() {
        return MathUtil.applyDeadband(super.getLeftTriggerAxis(), OperatorConstants.kDeadband);
    }

    @Override
    public double getRightTriggerAxis() {
        return MathUtil.applyDeadband(super.getRightTriggerAxis(), OperatorConstants.kDeadband);
    }
}
