package frc.robot.commands.testcommands;

import edu.wpi.first.wpilibj2.command.Command;

// Shared state for test commands
public class WheelTestContext {
    // Shared handle for the overall test sequence.
    public Command cancellableCommand = null;

    // Constructor
    public WheelTestContext() {
        reset();
    }

    // Reset context on the current test run.  But do NOT reset cancellableCommand,
    // since this object is reused across multiple test runs.
    public void reset() {
    }    
}
