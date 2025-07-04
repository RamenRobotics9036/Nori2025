package frc.robot.ramenlib.sim.simcommands.pretend;

/**
 * This command just sleeps for 1 second. But during that second it DOES
 * hold the dependencies on the subsystem, just like a real command would.
 */
public class PretendCommandNoSystem extends AbstractPretendCommand {
    /**
     * Constructor.
     */
    public PretendCommandNoSystem() {
        super(null);
    }

    @Override
    public String getSystemName() {
        return "No System";
    }
}
