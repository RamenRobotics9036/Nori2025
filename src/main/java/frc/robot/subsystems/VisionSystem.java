package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import frc.robot.Constants.VisionConstants;

public class VisionSystem {

    private static NetworkTable m_limelightTable = NetworkTableInstance.getDefault()
            .getTable(VisionConstants.limelightName);
    private static NetworkTableEntry m_tableX = m_limelightTable.getEntry("tx");
    private static NetworkTableEntry m_tableY = m_limelightTable.getEntry("ty");
    private static NetworkTableEntry m_tableArea = m_limelightTable.getEntry("ta");

    public static void initShuffleboad() {
        ShuffleboardTab tab = Shuffleboard.getTab("Vision");
        tab.addDouble("TX", () -> getTX());
        tab.addDouble("TY", () -> getTY());
        tab.addBoolean("Is Detecting", () -> isDetecting());
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
}
