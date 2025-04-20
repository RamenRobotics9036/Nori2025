package frc.robot.ramenlib.sim.armsimulation;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.simulation.DutyCycleEncoderSim;
import frc.robot.ramenlib.sim.simutil.RelativeEncoderSim;

public class IOArmSim implements IOArmSimInterface {
    private DutyCycleEncoderSim m_absEncoderSim;
    private RelativeEncoderSim m_relEncoderSim;
    private DoubleSupplier m_setpointSupplier;

    // Constructor
    public IOArmSim(
        DutyCycleEncoderSim absEncoderSim,
        RelativeEncoderSim relEncoderSim,
        DoubleSupplier setpointSupplier) {

        if (!RobotBase.isSimulation()) {
            System.out.println("ERROR: IOArmSim should only be instantiated in simulation mode.");
        }
        if (absEncoderSim == null) {
            System.out.println("ERROR: absEncoderSim is null");
        }
        if (relEncoderSim == null) {
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
    public void initPosition(double physicalAngleDegrees) {
        // Were initializing the position of the arm in simulation.
        m_absEncoderSim.set(Units.degreesToRadians(physicalAngleDegrees));

        // Reset the relative encoder to 0.0
        m_relEncoderSim.setPosition(0.0);
    }

    @Override
    public double getPhysicalSetpointDegrees() {
        return m_setpointSupplier.getAsDouble();
    }

    @Override
    public void setPhysicalOutputArmDegreesAbsolute(double physicalAngleDegrees) {
        // NOTE: Normally, encoders use Rotations for units.  But conversion factor
        // was configured on the encoders to use radians.
        double physicalAngleRads = Units.degreesToRadians(physicalAngleDegrees);

        double previousPhysicalRads = m_absEncoderSim.get();

        // Set new value on the absolute encoder sim object.
        m_absEncoderSim.set(physicalAngleRads);

        // Move relative encoder by the DELTA.
        double delta = physicalAngleRads - previousPhysicalRads;

        // Set it on the relative encoder sim object.
        m_relEncoderSim.setPosition(m_relEncoderSim.getPosition() + delta);
    }
}
