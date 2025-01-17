package frc.robot.subsystems;

import java.util.Optional;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
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
    private static AprilTagFieldLayout m_aprilTagLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2025Reefscape);

    private static Pose2d m_targetPose;
    private static Pose2d m_robotPose;

    public static void initShuffleboad() {
        ShuffleboardTab tab = Shuffleboard.getTab("Vision");
        tab.addDouble("TX", () -> getTX());
        tab.addDouble("TY", () -> getTY());
        tab.addBoolean("Is Detecting", () -> isDetecting());
        tab.addDouble("ID", () -> getID());
    }

    public static void updatePose() {
        m_targetPose = getTargetPoseCall();
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

    public static double getID() {
        return m_tableID.getDouble(0.0);
    }

    private static Pose2d getTargetPoseCall() {
        Optional<Pose3d> targetPose =  m_aprilTagLayout.getTagPose((int) getID());
        if (targetPose.isPresent()) {
            return targetPose.get().toPose2d();
        } else {
            return null;
        }
    }

    private static Pose2d getRobotPoseCall() {
        return new Pose2d();
    }

    public static Pose2d getTargetPose() {
        return m_targetPose;
    }

    public static Pose2d getRobotPose() {
        return m_robotPose;
    }
}
