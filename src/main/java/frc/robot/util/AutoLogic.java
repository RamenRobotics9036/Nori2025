package frc.robot.util;



import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.trajectory.PathPlannerTrajectory;
import com.pathplanner.lib.util.FileVersionException;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.subsystems.SwerveSubsystem;
import java.io.IOException;
import java.util.List;
import org.json.simple.parser.ParseException;

public class AutoLogic {
    public RobotContainer m_robotContainer;
public AutoLogic(RobotContainer robotContainer) {
    m_robotContainer = robotContainer;
}
  public static SendableChooser<String> autoPicker = new SendableChooser<String>();


  public static ShuffleboardTab tab = Shuffleboard.getTab("Autos");

  
  public static Command getAutoCommand(String autoName) {
    // System.out.println("Path name: " + pathName);
    // Load the path you want to follow using its name in the GUI
    try {

      return AutoBuilder.buildAuto(autoName);

    } catch (FileVersionException e) {
      // TODO: handle exception

      DriverStation.reportError("Ooofs: " + e.getMessage(), e.getStackTrace());
    }

    // Create a path following command using AutoBuilder. This will also trigger event markers.

    return Commands.none();
  }
  
  public static PathPlannerPath getPathData(String pathName) {
    // System.out.println("Path name: " + pathName);
    // Load the path you want to follow using its name in the GUI
    try {
      PathPlannerPath path = PathPlannerPath.fromPathFile(pathName);

      return path;

    } catch (IOException | ParseException | FileVersionException e) {
      // TODO: handle exception

      DriverStation.reportError("Ooofs: " + e.getMessage(), e.getStackTrace());
    }

    // Create a path following command using AutoBuilder. This will also trigger event markers.

    return null;
  }

  public void initShuffleBoard() {
    m_robotContainer.m_swerveDrive.setupPathPlanner();

    addAutoOptions();

    tab.add("Auto Selector", autoPicker)
        .withWidget(BuiltInWidgets.kComboBoxChooser)
        .withPosition(0, 0)
        .withSize(2, 1);

    tab.addString("Current Selected path", () -> autoPicker.getSelected());
    if (RobotState.isAutonomous()) {
      getAutoCommand(autoPicker.getSelected());
    }
    PathPlannerPath plannerPath = getPathData(autoPicker.getSelected());

    tab.addStringArray("Path Poses: ", () -> toStringArray(plannerPath.getPathPoses()));

    tab.addStringArray("Path Waypoints: ", () -> toStringArray(plannerPath.getWaypoints()));
  }

  public static <T> String[] toStringArray(List<T> dataList) {
    System.out.println("SIZE" + dataList.size());
    String[] data = new String[dataList.size()]; // TODO FIX INFINTELY REPEATING LIST

    for (int i = 0; i < dataList.size(); i++) {

      String addedData = dataList.get(i).toString();

      data[i] = "\n" + addedData;
    }

    return data;
  }

  public static PathPlannerTrajectory makeTrajectory(
      PathPlannerPath path,
      ChassisSpeeds startingSpeeds,
      Rotation2d startingRotation,
      RobotConfig config) {

    PathPlannerTrajectory trajectory =
        new PathPlannerTrajectory(path, startingSpeeds, startingRotation, config);

    return trajectory;
  }

  public static void addAutoOptions() {
    autoPicker.setDefaultOption("DEFAULT PATH", "Test Path");
    autoPicker.setDefaultOption("Test2 PATH", "Test2");
 
  }
}