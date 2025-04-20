package frc.robot.ramenlib.sim.simcommands.pretend;

import frc.robot.subsystems.IntakeSystem;

/**
 * This command just sleeps for 1 second. But during that second it DOES
 * hold the dependencies on the subsystem, just like a real command would.
 */
public class PretendCommandIntakeSystem extends AbstractPretendCommand {
    /**
     * Constructor.
     */
    public PretendCommandIntakeSystem(IntakeSystem intakeSystem) {
        super(intakeSystem);
    }

    @Override
    public String getSystemName() {
        return "IntakeSystem";
    }
}
