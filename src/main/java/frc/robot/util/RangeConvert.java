package frc.robot.util;

public class RangeConvert {
    private final double m_minPhysicalArmDegrees;
    private final double m_maxPhysicalArmDegrees;
    private final double m_minSimDegrees;
    private final double m_maxSimDegrees;
    
    public RangeConvert(
        double minPhysicalArmDegrees,
        double maxPhysicalArmDegrees,
        double minSimDegrees,
        double maxSimDegrees) {

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
        return (simDegrees - m_minSimDegrees) / (m_maxSimDegrees - m_minSimDegrees) 
                * (m_maxPhysicalArmDegrees - m_minPhysicalArmDegrees) + m_minPhysicalArmDegrees;
    }

    public double physicalToSim(double armDegrees) {
        if (armDegrees < m_minPhysicalArmDegrees || armDegrees > m_maxPhysicalArmDegrees) {
            System.out.println("ERROR: armDegrees is out of range");
        }

        // converts from physical arm degrees to simulation degrees
        return (armDegrees - m_minPhysicalArmDegrees) / (m_maxPhysicalArmDegrees - m_minPhysicalArmDegrees) 
                * (m_maxSimDegrees - m_minSimDegrees) + m_minSimDegrees;
    }
}
