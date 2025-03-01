package frc.robot.vision;

import java.util.Optional;

import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import frc.robot.Constants;
import frc.robot.Constants.VisionConstants;

public class VisionSystemSim implements VisionSystemInterface {
    private VisionSim m_visionSim;
    private Pose3d m_absoluteTargetPose = new Pose3d();
    private Pose3d m_relativeTargetPose = new Pose3d();
    private Pose2d m_robotPose = new Pose2d();

    // Constructor
    public VisionSystemSim(VisionSim visionSim) {
        m_visionSim = visionSim;
    }

    @Override
    public void updatePoses() {
        m_absoluteTargetPose = calcAbsoluteTargetPoseHelper();

        Pose3d robotPose3d = calcRobotPoseHelper();
        m_robotPose = robotPose3d.toPose2d();

        m_relativeTargetPose = calcRelativeTargetPoseHelper(
            robotPose3d,
            m_absoluteTargetPose);
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
    public int getID() {
        Optional<PhotonTrackedTarget> result = m_visionSim.getBestTarget();
        if (result.isEmpty()) {
            return 0;
        }

        int fiducialId = result.get().getFiducialId();
        if (fiducialId == -1) {
            return 0;
        }
        return fiducialId;
    }

    // This method will return an empty Pose3d if theres no valid target.
    private Pose3d calcAbsoluteTargetPoseHelper() {
        Optional<PhotonTrackedTarget> result = m_visionSim.getBestTarget();
        if (result.isEmpty()) {
            return new Pose3d();
        }

        PhotonTrackedTarget target = result.get();
        if (!VisionConstants.kTagLayout.getTagPose(target.getFiducialId()).isPresent()) {
            return new Pose3d();
        }

        return VisionConstants.kTagLayout.getTagPose(target.getFiducialId()).get();
    }

    private Pose3d calcRelativeTargetPoseHelper(Pose3d robotPose, Pose3d targetPose) {
        // Get the camera's pose in field coordinates
        Pose3d cameraPose = robotPose.transformBy(Constants.VisionSimConstants.kRobotToCam);

        // Transform the target's field pose into the camera's coordinate system
        return targetPose.relativeTo(cameraPose);
    }

    private Pose3d calcRobotPoseHelper() {
        Optional<PhotonTrackedTarget> result = m_visionSim.getBestTarget();
        if (result.isEmpty()) {
            return new Pose3d();
        }

        PhotonTrackedTarget target = result.get();
        if (!VisionConstants.kTagLayout.getTagPose(target.getFiducialId()).isPresent()) {
            return new Pose3d();
        }

        Transform3d cameraToRobot = Constants.VisionSimConstants.kRobotToCam.inverse();
        Pose3d robotPose = PhotonUtils.estimateFieldToRobotAprilTag(
            target.getBestCameraToTarget(),
            VisionConstants.kTagLayout.getTagPose(target.getFiducialId()).get(),
            cameraToRobot);

        return robotPose;
    }

    @Override
    public Pose3d getAbsoluteTargetPose() {
        return m_absoluteTargetPose;
    }

    // NOTE: This is in camera space coordinates
    @Override
    public Pose3d getRelativeTargetPose() {
        return m_relativeTargetPose;
    }

    @Override
    public Pose2d getRobotPose() {
        return m_robotPose;
    }
}
