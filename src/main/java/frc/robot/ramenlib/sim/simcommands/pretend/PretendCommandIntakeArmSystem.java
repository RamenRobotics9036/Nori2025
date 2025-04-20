package frc.robot.ramenlib.sim.simcommands.pretend;

import frc.robot.subsystems.IntakeArmSystem;

// This command just sleeps for 1 second.  But during that second it DOES
// hold the dependencies on the subsystem, just like a real command would.
public class PretendCommandIntakeArmSystem extends AbstractPretendCommand {
    public PretendCommandIntakeArmSystem(IntakeArmSystem armSystem) {
        super(armSystem);
    }

    @Override
    public String getSystemName() {
        return "IntakeArmSystem";
    }
}
