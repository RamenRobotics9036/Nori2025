package frc.robot.ramenlib.sim.simcommands.pretend;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;

/**
 * This command just sleeps for 1 second. But during that second it DOES
 * hold the dependencies on the subsystem, just like a real command would.
 */
public abstract class AbstractPretendCommand extends Command {
    private Timer m_timer = new Timer();
    private final boolean m_doPrint = false;

    /**
     * Constructs an AbstractPretendCommand with the specified subsystem.
     *
     * @param system The subsystem required by this command. Can be null.
     */
    public AbstractPretendCommand(Subsystem system) {
        if (system != null) {
            addRequirements(system);
        }
    }

    /**
     * Gets the name of the system associated with this command.
     *
     * @return The name of the system.
     */
    public abstract String getSystemName();

    @Override
    public void initialize() {
        m_timer.restart();
        if (m_doPrint) {
            System.out.println(
                "Pretending to run command on "
                    + getSystemName()
                    + " for 1 second.");
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
        if (m_doPrint) {
            System.out.println("Done pretending on " + getSystemName() + ".");
        }
    }
}
