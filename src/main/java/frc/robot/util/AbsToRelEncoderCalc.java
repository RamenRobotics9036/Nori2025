package frc.robot.util;

// Given an absolute encoder value as input, calculates the relative encoder value,
// which is a delta from the initial absolute encoder values.
public class AbsToRelEncoderCalc {
    private double m_offset;
    private double m_lastAbsoluteValue;
    private boolean m_hasSeenFirstValue;
    
    // Constructor
    public AbsToRelEncoderCalc() {
        m_offset = 0.0;
        m_lastAbsoluteValue = 0.0;
        m_hasSeenFirstValue = false;
    }

    public void reset() {
        // If no absolute values have been seen, do nothing.
        if (!m_hasSeenFirstValue) {
            return;
        }

        m_offset = m_lastAbsoluteValue;
    }

    public double getRelativePosition() {
        if (!m_hasSeenFirstValue) {
            return 0.0;
        }

        return m_lastAbsoluteValue - m_offset;
    }

    public void setAbsolutePosition(double absolutePosition) {
        if (!m_hasSeenFirstValue) {
            m_offset = absolutePosition;
            m_hasSeenFirstValue = true;
        }

        m_lastAbsoluteValue = absolutePosition;
    }
}
