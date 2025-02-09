package frc.robot.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import frc.robot.Constants.VisionConstants;

public class VisionSystem {

    private static NetworkTable m_limelightTable = NetworkTableInstance.getDefault()
            .getTable(VisionConstants.limelightName);
    private static  NetworkTableEntry m_tableX = m_limelightTable.getEntry("tx");
    private static NetworkTableEntry m_tableY = m_limelightTable.getEntry("ty");
    private static NetworkTableEntry m_tableArea = m_limelightTable.getEntry("ta");
    private static NetworkTableEntry m_tableID = m_limelightTable.getEntry("tid");

    private static Pose3d m_relativeTargetPose = new Pose3d();
    private static Pose2d m_robotPose = new Pose2d();

    private static AprilTagFieldLayout m_aprilTagLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2025Reefscape);

    public static void initShuffleboad() {
        ShuffleboardTab tab = Shuffleboard.getTab("Vision");
        tab.addDouble("TX", () -> getTX());
        tab.addDouble("TY", () -> getTY());
        tab.addBoolean("Is Detecting", () -> isDetecting());
        tab.addDouble("ID", () -> getID());

        tab.addDouble("April Tag Relative X", () -> m_relativeTargetPose.getX());
        tab.addDouble("April Tag Relative Y", () -> m_relativeTargetPose.getY());
        tab.addDouble("April Tag Relative Z", () -> m_relativeTargetPose.getZ());
        tab.addDouble("April Tag Relative Rot", () -> m_relativeTargetPose.getRotation().getAngle());
    }

    public static void setRobotPose(Pose2d newPose) {
        m_robotPose = newPose;
    }

    public static void changeRobotPose(Transform2d deltaPose) {
        m_robotPose = m_robotPose.plus(deltaPose);
    }

    public static void updatePose() {
        m_relativeTargetPose = getRelativeTargetPoseCall();
        m_robotPose = getRobotPoseCall();
    }

    public static double getTX() {
        return m_tableX.getDouble(0.0);
    }

    public static double getTY() {
        return m_tableY.getDouble(0.0);
    }

    public static double getTA() {
        return m_tableArea.getDouble(0.0);
    }

    public static boolean isDetecting() {
        return (getTX() + getTY() + getTA()) != 0;
    }

    public static int getID() {
        return (int) m_tableID.getInteger(0);
    }

    private static Pose3d getRelativeTargetPoseCall() {
        if (isDetecting()) {
            return LimelightHelpers.getTargetPose3d_CameraSpace(VisionConstants.limelightName);
        }
        return m_relativeTargetPose;
    }

    private static Pose2d getRobotPoseCall() {
        if (isDetecting()) {
            return LimelightHelpers.getBotPose2d_wpiBlue(VisionConstants.limelightName);
        }
        return m_robotPose;
    }

    public static Pose3d getRelativeTargetPose() {
        return m_relativeTargetPose;
    }

    public static Pose2d getRobotPose() {
        return m_robotPose;
    }

    public static Pose3d getAbsoluteTargetPose() {
        if (VisionSystem.isDetecting()) {
            return m_aprilTagLayout.getTagPose(getID()).get();
        }
        return new Pose3d();
    }
}
