package frc.robot.ramenlib.sim.armsimulation;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import frc.robot.ramenlib.sim.simutil.RangeConvert;

/**
 * The ArmDisplay class is responsible for visualizing the arm mechanism
 * in the simulation using WPILib's Mechanism2d.
 */
public class ArmDisplay {
    private static final Color8Bit m_defaultArmColor = new Color8Bit(Color.kYellow);
    private static final Color8Bit m_brokenArmColor = new Color8Bit(Color.kRed);
    private static final double m_epsilonBrokenDegrees = 2.0;
    private RangeConvert m_rangesPhysicalAndSim;
    private Mechanism2d m_mech2d;
    private MechanismLigament2d m_armLigament;

    /**
     * Constructor.
     */
    public ArmDisplay(RangeConvert rangesPhysicalAndSim) {
        m_rangesPhysicalAndSim = rangesPhysicalAndSim;

        // Create the arm display in ShuffleBoard
        m_mech2d = new Mechanism2d(60, 60);
        m_armLigament = createArmLigament(m_mech2d);
    }

    public Mechanism2d getMech2d() {
        return m_mech2d;
    }

    /**
     * Sets the angle of the arm in the simulation.
     *
     * @param physicalArmRads The angle of the physical arm in radians.
     */
    public void setAngle(double physicalArmRads) {
        double physicalArmDegrees = Units.radiansToDegrees(physicalArmRads);
        double simArmDegrees = m_rangesPhysicalAndSim.physicalToSim(physicalArmDegrees);

        Color8Bit armColor = isBroken(simArmDegrees) ? m_brokenArmColor : m_defaultArmColor;

        // We will only DISPLAY the arm as being between [min,max]
        simArmDegrees = MathUtil.clamp(
            simArmDegrees,
            m_rangesPhysicalAndSim.getMinSimDegrees(),
            m_rangesPhysicalAndSim.getMaxSimDegrees());

        m_armLigament.setAngle(simArmDegrees);
        m_armLigament.setColor(armColor);
    }

    // NOTE: We consider arm broken if it's very near max degrees.
    private boolean isBroken(double simArmDegrees) {
        return simArmDegrees <= m_rangesPhysicalAndSim.getMinSimDegrees() + m_epsilonBrokenDegrees
            || simArmDegrees >= m_rangesPhysicalAndSim.getMaxSimDegrees() - m_epsilonBrokenDegrees;
    }

    private MechanismLigament2d createArmLigament(Mechanism2d mech2d) {
        MechanismRoot2d armPivot = mech2d.getRoot("ArmPivot", 30, 30);

        @SuppressWarnings("unused")
        MechanismLigament2d armTower = armPivot
            .append(new MechanismLigament2d("ArmTower", 20, -90));
        MechanismLigament2d armLigament = armPivot.append(
            new MechanismLigament2d("Arm", 15, 0, 6, m_defaultArmColor));

        return armLigament;
    }
}
