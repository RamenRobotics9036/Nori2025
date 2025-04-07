package frc.robot.ramenutils.sim.simcommands.pretend;

import edu.wpi.first.wpilibj2.command.Command;

// This command just sleeps for 1 second.  But during that second it DOES
// hold the dependencies on the subsystem, just like a real command would.
public class UnexpectedCommand extends Command {
    private String m_name;
    private boolean m_isDone;
    private boolean m_throwException = true;

    public UnexpectedCommand(String name) {
        m_name = name;
        m_isDone = false;
    }

    @Override
    public void initialize() {
        String errStr = "Unexpected command run: " + m_name;
        if (m_throwException) {
            throw new UnsupportedOperationException(errStr);
        } else {
            System.out.println(errStr);
        }

        m_isDone = false;
    }

    @Override
    public void execute() {
        m_isDone = true;
    }

    @Override
    public boolean isFinished() {
        return m_isDone;
    }

    @Override
    public void end(boolean interrupted) {
    }
}
