package frc.robot.subsystems.VisionHelpers;

import edu.wpi.first.wpilibj.Timer;

public class ClockReal implements ClockInterface {
    @Override
    public double getTime() {
        return Timer.getFPGATimestamp();
    }    
}
