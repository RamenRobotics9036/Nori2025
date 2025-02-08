package frc.robot.vision;

import java.util.Optional;

import org.photonvision.targeting.PhotonTrackedTarget;

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
        tab.addDouble("TX", () -> getTX());
        tab.addDouble("TY", () -> getTY());
        tab.addBoolean("Is Detecting", () -> isDetecting());
        tab.addDouble("ID", () -> getID());
    }

    @Override
    public void updatePose() {
        // Empty implementation for simulation
    }

    @Override
    public double getTX() {
        Optional<PhotonTrackedTarget> result = m_visionSim.getBestTarget();
        if (result.isEmpty()) {
            return 0.0;
        }

        return result.get().getYaw();
    }

    @Override
    public double getTY() {
        Optional<PhotonTrackedTarget> result = m_visionSim.getBestTarget();
        if (result.isEmpty()) {
            return 0.0;
        }

        return result.get().getPitch();
    }

    @Override
    public double getTA() {
        Optional<PhotonTrackedTarget> result = m_visionSim.getBestTarget();
        if (result.isEmpty()) {
            return 0.0;
        }

        return result.get().getArea();
    }

    @Override
    public boolean isDetecting() {
        Optional<PhotonTrackedTarget> result = m_visionSim.getBestTarget();
        if (result.isEmpty()) {
            return false;
        }

        return true;
    }

    // Returns 0 if no ID
    @Override
    public double getID() {
        Optional<PhotonTrackedTarget> result = m_visionSim.getBestTarget();
        if (result.isEmpty()) {
            return 0.0;
        }

        int fiducialId = result.get().getFiducialId();
        if (fiducialId == -1) {
            return 0.0;
        }
        return (double)fiducialId;
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
