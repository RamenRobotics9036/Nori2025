package frc.robot.ramenutils.sim.simutil;

public class RangeConvert {
    private final double m_minPhysicalArmDegrees;
    private final double m_maxPhysicalArmDegrees;
    private final double m_minSimDegrees;
    private final double m_maxSimDegrees;
    private final boolean m_inverted;
    
    // NOTE: We allow for a buffer in the min and max degrees to allow for a little bit of wiggle
    // room in the simulation.  For example, the simulation has gravity pulling down on the arm, so
    // if the robot purposefully moves the arm to the lowwest point, the simulated arm will actually
    // droop a little bit below the lowest point.
    public RangeConvert(
        double minPhysicalArmDegrees,
        double maxPhysicalArmDegrees,
        double minSimDegrees,
        double maxSimDegrees,
        double bufferDegrees,
        boolean inverted) {

        if (minPhysicalArmDegrees < 0 || maxPhysicalArmDegrees < 0 || minSimDegrees < 0 || maxSimDegrees < 0) {
            System.out.println("ERROR: Degrees must be non-negative");
        }
        if (minPhysicalArmDegrees >= 360.0 || maxPhysicalArmDegrees >= 360.0 || minSimDegrees >= 360.0 || maxSimDegrees >= 360.0) {
            System.out.println("ERROR: Degrees must be less than 360");
        }
        if (minPhysicalArmDegrees >= maxPhysicalArmDegrees) {
            System.out.println("ERROR: m_minPhysicalArmDegrees must be less than m_maxPhysicalArmDegrees");
        }
        if (minSimDegrees >= maxSimDegrees) {
            System.out.println("ERROR: m_minSimDegrees must be less than m_maxSimDegrees");
        }
        this.m_minPhysicalArmDegrees = minPhysicalArmDegrees - bufferDegrees;
        this.m_maxPhysicalArmDegrees = maxPhysicalArmDegrees + bufferDegrees;
        this.m_minSimDegrees = minSimDegrees - bufferDegrees;
        this.m_maxSimDegrees = maxSimDegrees + bufferDegrees;
        this.m_inverted = inverted;
    }

    public RangeConvert(
        double minPhysicalArmDegrees,
        double maxPhysicalArmDegrees,
        double minSimDegrees,
        double maxSimDegrees,
        double bufferDegrees) {

        this(minPhysicalArmDegrees, maxPhysicalArmDegrees, minSimDegrees, maxSimDegrees, bufferDegrees, false);
    }

    public double getMinPhysicalArmDegrees() {
        return m_minPhysicalArmDegrees;
    }

    public double getMaxPhysicalArmDegrees() {
        return m_maxPhysicalArmDegrees;
    }

    public double getMinSimDegrees() {
        return m_minSimDegrees;
    }

    public double getMaxSimDegrees() {
        return m_maxSimDegrees;
    }

    public double simToPhysical(double simDegrees) {
        if (simDegrees < m_minSimDegrees || simDegrees > m_maxSimDegrees) {
            System.out.println("ERROR: simDegrees is out of range");
        }
        
        // converts from simulation degrees to physical arm degrees
        double result = (simDegrees - m_minSimDegrees) / (m_maxSimDegrees - m_minSimDegrees)
                * (m_maxPhysicalArmDegrees - m_minPhysicalArmDegrees) + m_minPhysicalArmDegrees;

        if (m_inverted) {
            result = m_maxPhysicalArmDegrees - result + m_minPhysicalArmDegrees;
        }

        return result;
    }

    public double physicalToSim(double physicalDegrees) {
        if (physicalDegrees < m_minPhysicalArmDegrees || physicalDegrees > m_maxPhysicalArmDegrees) {
            System.out.println("ERROR: armDegrees is out of range");
        }

        // converts from physical arm degrees to simulation degrees
        double result = (physicalDegrees - m_minPhysicalArmDegrees) / (m_maxPhysicalArmDegrees - m_minPhysicalArmDegrees)
                * (m_maxSimDegrees - m_minSimDegrees) + m_minSimDegrees;

        if (m_inverted) {
            result = m_maxSimDegrees - result + m_minSimDegrees;
        }

        return result;
    }
}
