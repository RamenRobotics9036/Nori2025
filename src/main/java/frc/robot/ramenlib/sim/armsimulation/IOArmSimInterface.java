package frc.robot.ramenlib.sim.armsimulation;

/** 
* This class is intended to give the SIMULATION implementation a
* simpler interface to use.
*/
@SuppressWarnings("AbbreviationAsWordInNameCheck")
public interface IOArmSimInterface {
    /**
     * Initializes the position of the arm in the simulation.
     *
     * @param physicalAngleDegrees The initial physical angle of the arm in degrees.
     */
    void initPosition(double physicalAngleDegrees);

    //
    // Inputs to the simulation
    //

    /**
     * Gets the physical setpoint angle of the arm in degrees.
     *
     * @return The physical setpoint angle in degrees.
     */
    double getPhysicalSetpointDegrees();

    //
    // Outputs that the simulation calculates
    //

    /**
     * Sets the physical output angle of the arm in degrees (absolute).
     *
     * @param physicalAngleDegrees The physical angle to set in degrees.
     */
    void setPhysicalOutputArmDegreesAbsolute(double physicalAngleDegrees);
}
