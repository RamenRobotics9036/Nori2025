package frc.robot.commands.pretend;

import frc.robot.subsystems.IntakeSystem;

// This command just sleeps for 1 second.  But during that second it DOES
// hold the dependencies on the subsystem, just like a real command would.
public class PretendCommandIntakeSystem extends AbstractPretendCommand {
    public PretendCommandIntakeSystem(IntakeSystem inttakeSystem) {
        super(inttakeSystem);
    }

    @Override
    public String getSystemName() {
        return "IntakeSystem";
    }
}
