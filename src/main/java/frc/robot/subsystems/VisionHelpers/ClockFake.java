package frc.robot.subsystems.VisionHelpers;

// We use a fake clock, since using the real clock would make the tests
// non-deterministic.
public class ClockFake implements ClockInterface {
    private double m_currentTime = 0.0;

    @Override
    public double getTime() {
        double result = m_currentTime;

        // Add a hundredth of a second
        m_currentTime += 0.01;

        return result;
    }
}
