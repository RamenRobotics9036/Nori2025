package frc.robot.sim.armsimulation;

// This class is intended to give the SIMULATION implementation a
// simpler interface to use.
public interface IOArmSimInterface {
    void initPosition(double physicalAngleDegrees);

    //
    // Inputs to the simulation
    //
    double getPhysicalSetpointDegrees();

    //
    // Outputs that the simulation calculates
    //
    void setPhysicalOutputArmDegreesAbsolute(double physicalAngleDegrees);
}
