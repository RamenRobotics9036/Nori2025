package frc.robot.commands.pretend;

import frc.robot.subsystems.ElevatorSystem;

// This command just sleeps for 1 second.  But during that second it DOES
// hold the dependencies on the subsystem, just like a real command would.
public class PretendCommandElevatorSystem extends AbstractPretendCommand {
    public PretendCommandElevatorSystem(ElevatorSystem elevatorSystem) {
        super(elevatorSystem);
    }

    @Override
    public String getSystemName() {
        return "ElevatorSystem";
    }
}
