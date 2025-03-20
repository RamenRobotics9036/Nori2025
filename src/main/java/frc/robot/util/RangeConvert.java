package frc.robot.util;

public class RangeConvert {
    private final double m_minPhysicalArmDegrees;
    private final double m_maxPhysicalArmDegrees;
    private final double m_minSimDegrees;
    private final double m_maxSimDegrees;
    private final boolean m_inverted;
    
    public RangeConvert(
        double minPhysicalArmDegrees,
        double maxPhysicalArmDegrees,
        double minSimDegrees,
        double maxSimDegrees,
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
        this.m_minPhysicalArmDegrees = minPhysicalArmDegrees;
        this.m_maxPhysicalArmDegrees = maxPhysicalArmDegrees;
        this.m_minSimDegrees = minSimDegrees;
        this.m_maxSimDegrees = maxSimDegrees;
        this.m_inverted = inverted;
    }

    public RangeConvert(
        double minPhysicalArmDegrees,
        double maxPhysicalArmDegrees,
        double minSimDegrees,
        double maxSimDegrees) {

        this(minPhysicalArmDegrees, maxPhysicalArmDegrees, minSimDegrees, maxSimDegrees, false);
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
