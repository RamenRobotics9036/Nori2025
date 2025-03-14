package frc.robot.commands.pretend;

import frc.robot.subsystems.OuttakeSystem;

// This command just sleeps for 1 second.  But during that second it DOES
// hold the dependencies on the subsystem, just like a real command would.
public class PretendCommandOuttakeSystem extends AbstractPretendCommand {
    public PretendCommandOuttakeSystem(OuttakeSystem outtakeSystem) {
        super(outtakeSystem);
    }

    @Override
    public String getSystemName() {
        return "OuttakeSystem";
    }
}
