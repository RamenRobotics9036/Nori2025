package frc.robot.subsystems.armsimulation;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.simulation.DutyCycleEncoderSim;
import frc.robot.util.AbsToRelEncoderCalc;
import frc.robot.util.RelativeEncoderSim;

public class IOArmSim implements IOArmSimInterface {
    private DutyCycleEncoderSim m_absEncoderSim;
    private RelativeEncoderSim m_relEncoderSim;
    private DoubleSupplier m_setpointSupplier;
    private AbsToRelEncoderCalc m_absToRelEncoderCalc = new AbsToRelEncoderCalc();

    // Constructor
    public IOArmSim(
        DutyCycleEncoderSim absEncoderSim,
        RelativeEncoderSim relEncoderSim,
        DoubleSupplier setpointSupplier) {

        if (!RobotBase.isSimulation()) {
            System.out.println("ERROR: IOArmSim should only be instantiated in simulation mode.");
        }
        if (m_absEncoderSim == null) {
            System.out.println("ERROR: absEncoderSim is null");
        }
        if (m_relEncoderSim == null) {
            System.out.println("ERROR: relEncoderSim is null");
        }
        if (setpointSupplier == null) {
            System.out.println("ERROR: setpointSupplier is null");
        }

        m_absEncoderSim = absEncoderSim;
        m_relEncoderSim = relEncoderSim;
        m_setpointSupplier = setpointSupplier;
    }

    @Override
    public double getSetpoint() {
        return m_setpointSupplier.getAsDouble();
    }

    @Override
    public void setOutputArmAngleAbsolute(double angleRads) {
        // Set it on the absolute encoder sim object.
        m_absEncoderSim.set(angleRads);

        // Update our relative encoder calculator.
        m_absToRelEncoderCalc.setAbsolutePosition(angleRads);

        // Set it on the relative encoder sim object.
        m_relEncoderSim.setPosition(m_absToRelEncoderCalc.getRelativePosition());
    }
}
