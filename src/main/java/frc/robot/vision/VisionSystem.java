package frc.robot.vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import frc.robot.Constants.VisionConstants;

public class VisionSystem implements VisionSystemInterface {

    private NetworkTable m_limelightTable = NetworkTableInstance.getDefault()
            .getTable(VisionConstants.limelightName);
    private NetworkTableEntry m_tableX = m_limelightTable.getEntry("tx");
    private NetworkTableEntry m_tableY = m_limelightTable.getEntry("ty");
    private NetworkTableEntry m_tableArea = m_limelightTable.getEntry("ta");
    private NetworkTableEntry m_tableID = m_limelightTable.getEntry("tid");

    private Pose3d m_targetPose = null;
    private Pose2d m_robotPose = null;

    // Constructor
    public VisionSystem() {
 
    }

    @Override
    public void initShuffleboad(ShuffleboardTab tab) {
        tab.addDouble("TX", () -> getTX());
        tab.addDouble("TY", () -> getTY());
        tab.addBoolean("Is Detecting", () -> isDetecting());
        tab.addDouble("ID", () -> getID());

        tab.addDouble("April Tag Relative X", () -> m_targetPose.getX());
        tab.addDouble("April Tag Relative Y", () -> m_targetPose.getY());
        tab.addDouble("April Tag Relative Z", () -> m_targetPose.getZ());
        tab.addDouble("April Tag Relative Rot", () -> m_targetPose.getRotation().getAngle());

    }

    @Override
    public void updatePose() {
        m_targetPose = getTargetPoseCall();
        m_robotPose = getRobotPoseCall();
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
    public double getID() {
        return m_tableID.getDouble(0.0);
    }

    private Pose3d getTargetPoseCall() {
        if (isDetecting()) {
            return LimelightHelpers.getTargetPose3d_CameraSpace(VisionConstants.limelightName);
        }
        return new Pose3d();
    }

    private Pose2d getRobotPoseCall() {
        if (isDetecting()) {
            return LimelightHelpers.getBotPose2d_wpiBlue(VisionConstants.limelightName);
        }
        return new Pose2d();
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
