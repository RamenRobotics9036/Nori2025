package frc.robot.commands.pretend;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;

// This command just sleeps for 1 second.  But during that second it DOES
// hold the dependencies on the subsystem, just like a real command would.
public abstract class AbstractPretendCommand extends Command {
    private Timer m_timer = new Timer();
    private final boolean doPrint = false;

    public AbstractPretendCommand(Subsystem system) {
        addRequirements(system);
    }

    public abstract String getSystemName();

    @Override
    public void initialize() {
        m_timer.restart();
        if (doPrint) {
            System.out.println("Pretending to run command on " + getSystemName() + " for 1 second.");
        }
    }

    @Override
    public void execute() {
    }

    @Override
    public boolean isFinished() {
        if (m_timer.get() > 1) {
            return true;
        }

        return false;
    }

    @Override
    public void end(boolean interrupted) {
        if (doPrint) {
            System.out.println("Done pretending on " + getSystemName() + ".");
        }
    }
}
