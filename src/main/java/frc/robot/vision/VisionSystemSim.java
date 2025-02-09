package frc.robot.vision;

import java.util.Optional;

import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import frc.robot.Constants;
import frc.robot.Constants.VisionSimConstants;

public class VisionSystemSim implements VisionSystemInterface {
    private VisionSim m_visionSim;
    private Pose3d m_targetPose3d = new Pose3d();
    private Pose2d m_robotPose2d = new Pose2d();
    private Pose3d m_targetPoseInCameraSpace3d = new Pose3d();

    // Constructor
    public VisionSystemSim(VisionSim visionSim) {
        m_visionSim = visionSim;
    }

    @Override
    public void updatePose() {
        m_targetPose3d = calcTargetPose3d();

        Pose3d robotPose3d = calcRobotPose3d();
        m_robotPose2d = robotPose3d.toPose2d();

        m_targetPoseInCameraSpace3d = calcTargetPoseInCameraSpace3d(
            robotPose3d,
            m_targetPose3d);
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

    private Pose3d calcTargetPose3d() {
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

    private Pose3d calcRobotPose3d() {
        Optional<PhotonTrackedTarget> result = m_visionSim.getBestTarget();
        if (result.isEmpty()) {
            return new Pose3d();
        }

        PhotonTrackedTarget target = result.get();
        if (!VisionSimConstants.kTagLayout.getTagPose(target.getFiducialId()).isPresent()) {
            return new Pose3d();
        }

        Transform3d cameraToRobot = Constants.VisionSimConstants.kRobotToCam.inverse();
        Pose3d robotPose = PhotonUtils.estimateFieldToRobotAprilTag(
            target.getBestCameraToTarget(),
            VisionSimConstants.kTagLayout.getTagPose(target.getFiducialId()).get(),
            cameraToRobot);

        return robotPose;
    }

    private Pose3d calcTargetPoseInCameraSpace3d(Pose3d robotPose, Pose3d targetPose) {
        // Get the camera's pose in field coordinates
        Pose3d cameraPose = robotPose.transformBy(Constants.VisionSimConstants.kRobotToCam);

        // Transform the target's field pose into the camera's coordinate system
        return targetPose.relativeTo(cameraPose);
    }

    // NOTE: This is expected in camera space
    @Override
    public Pose3d getTargetPose() {
        return m_targetPoseInCameraSpace3d;
    }

    @Override
    public Pose2d getRobotPose() {
        return m_robotPose2d;
    }
}
