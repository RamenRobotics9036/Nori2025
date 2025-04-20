package frc.robot.ramenlib.sim.simutil;

import com.revrobotics.RelativeEncoder;
import edu.wpi.first.wpilibj.RobotBase;

/**
 * Rev robotics doesnt have a simulation class for RelativeEncoder. However,
 * a common pattern in FRC is to only set the RelativeEncoder's current position
 * by setting it on the RelativeEncoderSim object. This way, the simulation never
 * directly touches the RelativeEncoder object, which is a bit cleaner.
 */
public class RelativeEncoderSim {
    private RelativeEncoder m_relEncoder;

    /**
     * Constructor.
     */
    public RelativeEncoderSim(RelativeEncoder relEncoder) {
        // This entire class should only be instantiated when we're under simulation.
        // But just in-case someone tries to instantiate it otherwise, we do an extra check here.
        if (RobotBase.isSimulation()) {
            m_relEncoder = relEncoder;
        }
        else {
            System.out.println(
                "ERROR: RelativeEncoderSim should only "
                    + "be instantiated in simulation mode.");
            m_relEncoder = null;
        }
    }

    public double getPosition() {
        return m_relEncoder.getPosition();
    }

    /**
     * Sets the position of the RelativeEncoder in simulation.
     *
     * @param position The desired position to set.
     */
    public void setPosition(double position) {
        m_relEncoder.setPosition(position);
    }
}
