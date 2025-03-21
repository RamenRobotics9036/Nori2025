package frc.robot.sim.armsimulation;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units; 
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.robot.sim.simutil.RangeConvert;

// This class internally simulates an arm using a min-simulation-degrees to max-simulation-degrees
// range.  This was chosen for one primary reason:
//   1. Since the arm simulation uses gravity, its important that it simulate the exact
//      range that we DISPLAY in mech2d.  Otherwise, the gravity will be wrong.
//
// Net-net: IOArmSim is always dealing in the PHYSICAL range of the arm.  But the arm
// simulation is always dealing with the SIMULATION range.
//
// NOTE: When the robot is PHYSICALLY running, we re-use the mech2d to display the current
// arm position.  So even in this case, we convert the phyical arm position to the
// simulated range.
//
public class ArmSimulation {
    private IOArmSimInterface m_ioArmSimInterface;
 
    private PIDController m_pidController = new PIDController(1.0, 0.0, 0.0);

    private SingleJointedArmSim m_armSim;

    private RangeConvert m_rangesPhysicalAndSim;
    
    // Constructor
    public ArmSimulation(
        IOArmSimInterface ioArmSimInterface,
        RangeConvert rangesPhysicalAndSim) {

        if (!RobotBase.isSimulation()) {
            System.out.println("ERROR: ArmSimulation is only available in simulation mode.");
            return;
        }

        m_rangesPhysicalAndSim = rangesPhysicalAndSim;
        if (m_rangesPhysicalAndSim.getMinPhysicalArmDegrees() >= m_rangesPhysicalAndSim.getMaxPhysicalArmDegrees()) {
            throw new IllegalArgumentException("getMinPhysicalArmDegrees must be less than getMaxPhysicalArmDegrees");
        }
        if (m_rangesPhysicalAndSim.getMinSimDegrees() >= m_rangesPhysicalAndSim.getMaxSimDegrees()) {
            throw new IllegalArgumentException("getMinSimDegrees must be less than getMaxSimDegrees");
        }

        m_ioArmSimInterface = ioArmSimInterface;

        m_armSim = createSingleJointedArmSim();

        initArmPosition();
    }

    // In simulation, the arm encoder is at 0, rather than at some reasonable
    // arm position half-way up.  So we explicitely set the arm to a half-way
    // point here.
    private void initArmPosition() {
        double halfwaySimDegrees = (m_rangesPhysicalAndSim.getMinSimDegrees() + m_rangesPhysicalAndSim.getMaxSimDegrees()) / 2.0;

        double halfwayPhysicalDegrees = m_rangesPhysicalAndSim.simToPhysical(halfwaySimDegrees);

        m_ioArmSimInterface.initPosition(halfwayPhysicalDegrees);
    }

    public void simulationPeriodic() {
        // Read the setpoint from the IO 
        double physicalDesiredAngleDegrees = m_ioArmSimInterface.getPhysicalSetpointDegrees();
        double simDesiredAngleDegrees = m_rangesPhysicalAndSim.physicalToSim(physicalDesiredAngleDegrees);
        // System.out.println("##### Arm setpoint degrees = " + simDesiredAngleDegrees);

        double simDesiredAngleRads = Units.degreesToRadians(simDesiredAngleDegrees);
        double currentAngleRads = m_armSim.getAngleRads();

        // Use PID controller to get motor voltage.
        //
        // NOTE: The PID Controller assumes Radians for how it was tuned.  I didnt
        // think it would matter, but when I tried using Degrees, the arm jittered around.
        double simOutput = m_pidController.calculate(currentAngleRads, simDesiredAngleRads);
        double motorVolts = MathUtil.clamp(simOutput, -1.0, 1.0) * 12.0;

        // Run the simulation with a particular motor voltage
        m_armSim.setInput(motorVolts);
        m_armSim.update(0.02);

        // Set the output
        double newSimAngleDegrees = Units.radiansToDegrees(m_armSim.getAngleRads());

        // Write the new position into the absolute encoder
        double newPhysiclAngleDegrees = m_rangesPhysicalAndSim.simToPhysical(newSimAngleDegrees);
        m_ioArmSimInterface.setPhysicalOutputArmDegreesAbsolute(newPhysiclAngleDegrees);
    }

    private SingleJointedArmSim createSingleJointedArmSim() {
        // The arm gearbox represents a gearbox containing two Vex 775pro motors. 
        DCMotor armGearbox = DCMotor.getVex775Pro(2);

        double kArmReduction = 200;
        double kArmLength = Units.inchesToMeters(20);
        double kArmMass = 4.0; // Kilograms
 
        // distance per pulse = (angle per revolution) / (pulses per revolution)
        //  = (2 * PI rads) / (4096 pulses)
        double kArmEncoderDistPerPulse = 2.0 * Math.PI / 4096;
 
        return new SingleJointedArmSim( 
            armGearbox,
            kArmReduction, 
            SingleJointedArmSim.estimateMOI(kArmLength, kArmMass),
            kArmLength,
            Units.degreesToRadians(m_rangesPhysicalAndSim.getMinSimDegrees()),
            Units.degreesToRadians(m_rangesPhysicalAndSim.getMaxSimDegrees()),
            true,
            0,
            kArmEncoderDistPerPulse,
            0.0 // Add noise with a std-dev of 1 tick
        ); 
    }
}
