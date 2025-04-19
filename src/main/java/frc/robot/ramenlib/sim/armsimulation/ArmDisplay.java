package frc.robot.ramenlib.sim.armsimulation;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import frc.robot.ramenlib.sim.simutil.RangeConvert;

public class ArmDisplay {
    private RangeConvert m_rangesPhysicalAndSim;
    private Mechanism2d m_mech2d;
    private MechanismLigament2d m_armLigament;

    // Constructor
    public ArmDisplay(RangeConvert rangesPhysicalAndSim) {
        m_rangesPhysicalAndSim = rangesPhysicalAndSim;

        // Create the arm display in ShuffleBoard
        m_mech2d = new Mechanism2d(60, 60);
        m_armLigament = createArmLigament(m_mech2d);
    }

    public Mechanism2d getMech2d() {
        return m_mech2d;
    }

    // This takes as input the angle of the physical arm in radians.
    public void setAngle(double physicalArmRads) {
        double physicalArmDegrees = Units.radiansToDegrees(physicalArmRads);
        double simArmDegrees = m_rangesPhysicalAndSim.physicalToSim(physicalArmDegrees);
        m_armLigament.setAngle(simArmDegrees); 
    }

    private MechanismLigament2d createArmLigament(Mechanism2d mech2d) {
        MechanismRoot2d armPivot = mech2d.getRoot("ArmPivot", 30, 30); 
        MechanismLigament2d armTower = armPivot.append(new MechanismLigament2d("ArmTower", 20, -90));
        MechanismLigament2d armLigament = armPivot.append( 
            new MechanismLigament2d("Arm", 15, 0, 6, new Color8Bit(Color.kYellow)) 
        );

        return armLigament;
    }
}
