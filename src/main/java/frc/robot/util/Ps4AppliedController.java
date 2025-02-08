package frc.robot.util;

import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * Abstraction for PS4 controller.
 */
public class Ps4AppliedController extends CommandPS4Controller
    implements IAppliedController {

    /**
     * Constructor.
     */
    public Ps4AppliedController(int port) {
        super(port);
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
        throw new UnsupportedOperationException("Not implemented yet.");
    }

    @Override
    public double getRightTriggerAxis() {
        throw new UnsupportedOperationException("Not implemented yet.");
    }

    @Override
    public Trigger start() {
        return super.options();
    }

    @Override
    public Trigger a() {
        return super.cross();
    }

    @Override
    public Trigger b() {
        return super.circle();
    }
}
