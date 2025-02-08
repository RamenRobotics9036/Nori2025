package frc.robot.util;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.CommandConstants;
import frc.robot.Constants.OperatorConstants;

/**
 * AppliedController.
 */
public class CommandAppliedController extends CommandXboxController
    implements IAppliedController {

    /**
     * Constructor.
     */
    public CommandAppliedController(int port) {
        super(port);
    }

    /**
     * Factory.
     */
    public static IAppliedController createInstance(int port) {
        if (OperatorConstants.kIsXbox) {
            return new CommandAppliedController(port);
        } else {
            return new Ps4AppliedController(port);
        }
    }

    /**
     * Overrides.
     */

    @Override
    public double getLeftY() {
        return ControllerUtilities.adjust(super.getLeftY());
    }

    @Override
    public double getRightY() {
        return ControllerUtilities.adjust(super.getRightY());
    }

    @Override
    public double getLeftX() {
        return ControllerUtilities.adjust(super.getLeftX());
    }

    @Override
    public double getRightX() {
        return ControllerUtilities.adjust(super.getRightX());
    }

    @Override
    public double getLeftTriggerAxis() {
        return MathUtil.applyDeadband(super.getLeftTriggerAxis(), OperatorConstants.kDeadband);
    }

    @Override
    public double getRightTriggerAxis() {
        return MathUtil.applyDeadband(super.getRightTriggerAxis(), OperatorConstants.kDeadband);
    }

    @Override
    public Trigger start() {
        return super.start();
    }

    @Override
    public Trigger a() {
        return super.a();
    }

    @Override
    public Trigger b() {
        return super.b();
    }
}
