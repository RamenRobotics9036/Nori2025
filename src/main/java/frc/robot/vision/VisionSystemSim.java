package frc.robot.vision;

import java.util.Optional;

import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import frc.robot.Constants;
import frc.robot.Constants.VisionSimConstants;

public class VisionSystemSim implements VisionSystemInterface {
    private VisionSim m_visionSim;
    private Pose3d m_targetPose = new Pose3d();
    private Pose2d m_robotPose = new Pose2d();

    // Constructor
    public VisionSystemSim(VisionSim visionSim) {
        m_visionSim = visionSim;
    }

    @Override
    public void initShuffleboad(ShuffleboardTab tab) {
        // $TODO - Move this out of this class
        tab.addDouble("TX", () -> getTX());
        tab.addDouble("TY", () -> getTY());
        tab.addBoolean("Is Detecting", () -> isDetecting());
        tab.addDouble("ID", () -> getID());

        // $TODO - This seems wrong
        tab.addDouble("April Tag Relative X", () -> m_targetPose.getX());
        tab.addDouble("April Tag Relative Y", () -> m_targetPose.getY());
        tab.addDouble("April Tag Relative Z", () -> m_targetPose.getZ());
        tab.addDouble("April Tag Relative Rot", () -> m_targetPose.getRotation().getAngle());
    }

    @Override
    public void updatePose() {
        m_targetPose = calcTargetPose();
        m_robotPose = calcRobotPose();
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

    private Pose3d calcTargetPose() {
        Optional<PhotonTrackedTarget> result = m_visionSim.getBestTarget();
        if (result.isEmpty()) {
            return new Pose3d();
        }

        PhotonTrackedTarget target = result.get();
        if (!VisionSimConstants.kTagLayout.getTagPose(target.getFiducialId()).isPresent()) {
            return new Pose3d();
        }

        return VisionSimConstants.kTagLayout.getTagPose(target.getFiducialId()).get();
    }

    private Pose2d calcRobotPose() {
        Optional<PhotonTrackedTarget> result = m_visionSim.getBestTarget();
        if (result.isEmpty()) {
            return new Pose2d();
        }

        PhotonTrackedTarget target = result.get();
        if (!VisionSimConstants.kTagLayout.getTagPose(target.getFiducialId()).isPresent()) {
            return new Pose2d();
        }

        Transform3d cameraToRobot = Constants.VisionSimConstants.kRobotToCam.inverse();
        Pose3d robotPose = PhotonUtils.estimateFieldToRobotAprilTag(
            target.getBestCameraToTarget(),
            VisionSimConstants.kTagLayout.getTagPose(target.getFiducialId()).get(),
            cameraToRobot);

        // Convert the 3d pose to a 2d pose
        return robotPose.toPose2d();
    }

    @Override
    public Pose3d getTargetPose() {
        return m_targetPose;
    }

    @Override
    public Pose2d getRobotPose() {
        return m_robotPose;
    }
}
