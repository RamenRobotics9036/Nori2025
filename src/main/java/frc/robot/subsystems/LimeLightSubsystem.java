package frc.robot.subsystems;


import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import java.util.Map;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.VisionConstants;

public class LimeLightSubsystem extends SubsystemBase {
    

    // Add these fields to store the Shuffleboard entries
    private GenericEntry sbXDisplacement;
    private GenericEntry sbYDisplacement;
    private GenericEntry sbXTangent;
    private GenericEntry sbYTangent;
    private GenericEntry sbDistanceX;
    private GenericEntry sbDistanceY;
    private GenericEntry sbArea;
    private GenericEntry sbIsDetecting;
    private GenericEntry sbID;
    private GenericEntry sbY;
    private GenericEntry sbX;

    private final double limelightMountAngleRadiansY = VisionConstants.limelightMountAngleRadiansY;
    private final double limelightMountAngleRadiansX = VisionConstants.limelightMountAngleRadiansX;

    private final double limelightLensHeightMeters = VisionConstants.limelightLensHeightMeters;
    private final double aprilTagHeightMeters = VisionConstants.aprilTagHeightMeters;

    private final double EPSILON = 0.0000001;

    private NetworkTable limelightTable = NetworkTableInstance.getDefault()
    .getTable(VisionConstants.limelightName);

    private NetworkTableEntry tableX = limelightTable.getEntry("tx");
    private NetworkTableEntry tableY = limelightTable.getEntry("ty");
    private NetworkTableEntry tableArea = limelightTable.getEntry("ta");
    private NetworkTableEntry tableID = limelightTable.getEntry("tid");

    public LimeLightSubsystem(){
        initializeShuffleboard();
    }

    private void initializeShuffleboard() {
        ShuffleboardTab visionTab = Shuffleboard.getTab("Vision");
        
        // Initialize the entries
        sbXDisplacement = visionTab.add("X Displacement", getXRadians())
            .withWidget(BuiltInWidgets.kNumberBar)
            .withPosition(0, 0)
            .withProperties(Map.of("min", -Math.PI/2, "max", Math.PI/2))
            .getEntry();
        
        sbYDisplacement = visionTab.add("Y Displacement", getYRadians())
            .withWidget(BuiltInWidgets.kNumberBar)
            .withPosition(0, 1)
            .withProperties(Map.of("min", -Math.PI/2, "max", Math.PI/2))
            .getEntry();
        
        sbXTangent = visionTab.add("X tangent", Math.tan(getXRadians()))
            .withWidget(BuiltInWidgets.kNumberBar)
            .withPosition(0, 2)
            .withProperties(Map.of("min", -5, "max", 5))
            .getEntry();
        
        sbYTangent = visionTab.add("Y tangent", Math.tan(getYRadians()))
            .withWidget(BuiltInWidgets.kNumberBar)
            .withPosition(0, 3)
            .withProperties(Map.of("min", -5, "max", 5))
            .getEntry();
        
        sbX = visionTab.add("X Degrees", getX())
            .withWidget(BuiltInWidgets.kNumberBar)
            .withPosition(4, 2)
            .withProperties(Map.of("min", -90, "max", 90))
            .getEntry();
        
        sbY = visionTab.add("Y Degrees", getY())
            .withWidget(BuiltInWidgets.kNumberBar)
            .withPosition(4, 3)
            .withProperties(Map.of("min", -90, "max", 90))
            .getEntry();
        
        sbDistanceX = visionTab.add("Distance Meters X", getDistanceMetersX())
            .withWidget(BuiltInWidgets.kNumberBar)
            .withPosition(2, 0)
            .withProperties(Map.of("min", 0, "max", 10))
            .getEntry();
        
        sbDistanceY = visionTab.add("Distance Meters Y", getDistanceMetersY())
            .withWidget(BuiltInWidgets.kNumberBar)
            .withPosition(2, 1)
            .withProperties(Map.of("min", 0, "max", 10))
            .getEntry();
        
        sbArea = visionTab.add("Area", getArea())
            .withWidget(BuiltInWidgets.kNumberBar)
            .withPosition(4, 0)
            .withProperties(Map.of("min", 0, "max", 100))
            .getEntry();
        
        sbIsDetecting = visionTab.add("Is Detecting", isDetected())
            .withWidget(BuiltInWidgets.kBooleanBox)
            .withPosition(4, 1)
            .getEntry();
        
        sbID = visionTab.add("ID", getID())
            .withWidget(BuiltInWidgets.kTextView)
            .withPosition(2, 2)
            .getEntry();
    }
    /**
     * X angle, left-right, from April tag. X cross-hair angle.
     */
    public double getX() {
        return tableX.getDouble(0);
    }

    /**
     * Y angle, up-down, to April tag. Y cross-hair angle.
     */
    public double getY() {
        return tableY.getDouble(0);
    }

    public double getXRadians() {
        return Math.toRadians(getX());
    }

    public double getYRadians() {
        return Math.toRadians(getY());
    }

    /**
     * Area of April tag in view.
     */
    public double getArea() {
        return tableArea.getDouble(0);
    }

    public boolean isDetected() {
        return getX() + getY() + getArea() != 0; // if there is any value, then you know it sees a value
    }

    public double getID() {
        return tableID.getDouble(0);
    }

    public boolean isDetectedIDValid() {
        double myID = getID();
        if (VisionConstants.targetedIDList.contains(myID)) {
            return true;
        }
        else {
            return false;
        }
    }

    /**
     * Distance to April tag in meters Y. (THIS IS DEF BROKEN FROM LOOKING AT THE NUMBERS)
     */
    public double getDistanceMetersY() {
        double angleToGoalRadians = limelightMountAngleRadiansY + getYRadians();
        double distanceFromLimelightToGoalMeters = (aprilTagHeightMeters
                - limelightLensHeightMeters) / (Math.tan(angleToGoalRadians) + EPSILON);
        return distanceFromLimelightToGoalMeters < 25 ? distanceFromLimelightToGoalMeters : 0; // Sometimes it will put a very large number when it doesn't detect something, so it
        // it makes sure it is within range
    }

    /**
     * Distance to April tag in meters X.
     */
    public double getDistanceMetersX() {
        double angleToGoalRadians = limelightMountAngleRadiansX + getXRadians();
        double distanceFromLimelightToGoalMeters = getDistanceMetersY()
                * Math.tan(angleToGoalRadians);
        return distanceFromLimelightToGoalMeters;
    }

    @Override
    public void periodic() {
        // Update Shuffleboard values
        sbXDisplacement.setDouble(getXRadians());
        sbYDisplacement.setDouble(getYRadians());
        sbXTangent.setDouble(Math.tan(getXRadians()));
        sbYTangent.setDouble(Math.tan(getYRadians()));
        sbX.setDouble(getX());
        sbY.setDouble(getY());
        sbDistanceX.setDouble(getDistanceMetersX());
        sbDistanceY.setDouble(getDistanceMetersY());
        sbArea.setDouble(getArea());
        sbIsDetecting.setBoolean(isDetected());
        sbID.setDouble(getID());
    }
  
    public void simulationPeriodic() {
      // This method will be called once per scheduler run during simulation
    }
}


