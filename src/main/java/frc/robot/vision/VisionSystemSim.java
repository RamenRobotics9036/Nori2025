package frc.robot.vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

public class VisionSystemSim implements VisionSystemInterface {
    VisionSim m_visionSim;

    // Constructor
    public VisionSystemSim(VisionSim visionSim) {
        m_visionSim = visionSim;
    }

    @Override
    public void initShuffleboad(ShuffleboardTab tab) {
        // Empty implementation for simulation
    }

    @Override
    public void updatePose() {
        // Empty implementation for simulation
    }

    @Override
    public double getTX() {
        return 0.0;
    }

    @Override
    public double getTY() {
        return 0.0;
    }

    @Override
    public double getTA() {
        return 0.0;
    }

    @Override
    public boolean isDetecting() {
        return false;
    }

    @Override
    public double getID() {
        return 0.0;
    }

    @Override
    public Pose3d getTargetPose() {
        return new Pose3d();
    }

    @Override
    public Pose2d getRobotPose() {
        return new Pose2d();
    }
}
