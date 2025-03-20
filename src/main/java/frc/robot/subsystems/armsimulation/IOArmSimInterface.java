package frc.robot.subsystems.armsimulation;

// This class is intended to give the SIMULATION implementation a
// simpler interface to use.
public interface IOArmSimInterface {
    void initPhysicalArmDegreesAbsolute(double physicalAngleDegrees);

    //
    // Inputs to the simulation
    //
    double getPhysicalSetpointDegrees();

    //
    // Outputs that the simulation calculates
    //
    void setPhysicalOutputArmDegreesAbsolute(double physicalAngleDegrees);
}
