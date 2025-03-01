package frc.robot.vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import frc.robot.Constants.VisionConstants;

public class VisionSystem implements VisionSystemInterface {

    private NetworkTable m_limelightTable = NetworkTableInstance.getDefault()
            .getTable(VisionConstants.limelightName);
    private NetworkTableEntry m_tableX = m_limelightTable.getEntry("tx");
    private NetworkTableEntry m_tableY = m_limelightTable.getEntry("ty");
    private NetworkTableEntry m_tableArea = m_limelightTable.getEntry("ta");
    private NetworkTableEntry m_tableID = m_limelightTable.getEntry("tid");

    private Pose3d m_absoluteTargetPose = new Pose3d();
    private Pose3d m_relativeTargetPose = new Pose3d();
    private Pose2d m_robotPose = new Pose2d();

    // Constructor
    public VisionSystem() {
        // Empty
        initShuffleboad();
    }

    public void initShuffleboad() {
        ShuffleboardTab tab = Shuffleboard.getTab("Vision");
        tab.addDouble("Robot Pose X", () -> getRobotPose().getX());
        tab.addDouble("Robot Pose Y", () -> getRobotPose().getY());
        tab.addDouble("Robot Pose Rot", () -> getRobotPose().getRotation().getDegrees());
    }

    @Override
    public void updatePoses() {
        m_absoluteTargetPose = calcAbsoluteTargetPoseHelper();
        m_relativeTargetPose = calcRelativeTargetPoseHelper();
        m_robotPose = calcRobotPoseHelper();
    }

    @Override
    public double getTX() {
        return m_tableX.getDouble(0.0);
    }

    @Override
    public double getTY() {
        return m_tableY.getDouble(0.0);
    }

    @Override
    public double getTA() {
        return m_tableArea.getDouble(0.0);
    }

    @Override
    public boolean isDetecting() {
        return (getTX() + getTY() + getTA()) != 0;
    }

    @Override
    public int getID() {
        return (int)m_tableID.getDouble(0.0);
    }

    private Pose3d calcAbsoluteTargetPoseHelper() {
        if (isDetecting()) {
            if (VisionConstants.kTagLayout.getTagPose(getID()).isPresent()) {
                return VisionConstants.kTagLayout.getTagPose(getID()).get();
            }
        }
        return new Pose3d();
    }

    private Pose3d calcRelativeTargetPoseHelper() {
        if (isDetecting()) {
            return LimelightHelpers.getTargetPose3d_CameraSpace(VisionConstants.limelightName);
        }
        return new Pose3d();
    }

    private Pose2d calcRobotPoseHelper() {
        if (isDetecting()) {
            return LimelightHelpers.getBotPoseEstimate_wpiBlue(VisionConstants.limelightName).pose;
        }
        return new Pose2d();
    }

    @Override
    public Pose3d getAbsoluteTargetPose() {
        return m_absoluteTargetPose;
    }

    @Override
    public Pose3d getRelativeTargetPose() {
        return m_relativeTargetPose;
    }

    @Override
    public Pose2d getRobotPose() {
        return m_robotPose;
    }
}
