package frc.robot.subsystems.armsimulation;

// This class is intended to give the SIMULATION implementation a
// simpler interface to use.
public interface IOArmSimInterface {

    //
    // Inputs to the simulation
    //
    double getSetpoint();

    //
    // Outputs that the simulation calculates
    //
    void setOutputArmAngle(double angleRads);
    void setOutputArmAngleRelative(double angleRads);
}
