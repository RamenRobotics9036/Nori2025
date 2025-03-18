package frc.robot.subsystems.armsimulation;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units; 
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;

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
    private Mechanism2d m_mech2d;
    private MechanismLigament2d m_armLigament; 

    // Constructor
    public ArmSimulation(IOArmSimInterface ioArmSimInterface) {
        if (!RobotBase.isSimulation()) {
            System.out.println("ERROR: ArmSimulation is only available in simulation mode.");
            return;
        }

        m_ioArmSimInterface = ioArmSimInterface;

        m_armSim = createSingleJointedArmSim();

        // Create the arm display in ShuffleBoard
        m_mech2d = new Mechanism2d(60, 60);
        m_armLigament = createArmLigament(m_mech2d);
    }

    public void simulationPeriodic() {
        // Read the setpoint from the IO 
        double desiredAngleDegrees = m_ioArmSimInterface.getSetpointDegrees();
        //System.out.println("##### Arm setpoint=" + desiredAngleDegrees);

        double desiredAngleRads = Units.degreesToRadians(desiredAngleDegrees);
        double currentAngleRads = m_armSim.getAngleRads();

        // Use PID controller to get motor voltage.
        //
        // NOTE: The PID Controller assumes Radians for how it was tuned.  I didnt
        // think it would matter, but when I tried using Degrees, the arm jittered around.
        double simOutput = m_pidController.calculate(currentAngleRads, desiredAngleRads);
        double motorVolts = MathUtil.clamp(simOutput, -1.0, 1.0) * 12.0;

        // Run the simulation with a particular motor voltage
        m_armSim.setInput(motorVolts);
        m_armSim.update(0.02);

        // Set the output
        double newAngleDegrees = Units.radiansToDegrees(m_armSim.getAngleRads());
        m_armLigament.setAngle(newAngleDegrees);

        // Write the new position into the absolute encoder
        m_ioArmSimInterface.setOutputArmDegreesAbsolute(newAngleDegrees);
    }

    public Mechanism2d getMech2d() {
        return m_mech2d;
    }

    private SingleJointedArmSim createSingleJointedArmSim() {
        // The arm gearbox represents a gearbox containing two Vex 775pro motors. 
        DCMotor armGearbox = DCMotor.getVex775Pro(2);

        double kArmReduction = 200;
        double kArmLength = Units.inchesToMeters(20);
        double kArmMass = 4.0; // Kilograms
        double kMinAngleRads = Units.degreesToRadians(-45);
        double kMaxAngleRads = Units.degreesToRadians(45);
 
        // distance per pulse = (angle per revolution) / (pulses per revolution)
        //  = (2 * PI rads) / (4096 pulses)
        double kArmEncoderDistPerPulse = 2.0 * Math.PI / 4096;
 
        return new SingleJointedArmSim( 
            armGearbox,
            kArmReduction, 
            SingleJointedArmSim.estimateMOI(kArmLength, kArmMass),
            kArmLength,
            kMinAngleRads,
            kMaxAngleRads,
            true,
            0,
            kArmEncoderDistPerPulse,
            0.0 // Add noise with a std-dev of 1 tick
        ); 
    }

    private MechanismLigament2d createArmLigament(Mechanism2d mech2d) {
        MechanismRoot2d armPivot = mech2d.getRoot("ArmPivot", 30, 30); 
        MechanismLigament2d armTower = armPivot.append(new MechanismLigament2d("ArmTower", 20, -90));
        MechanismLigament2d armLigament = armPivot.append( 
            new MechanismLigament2d("Arm", 15, 0, 6, new Color8Bit(Color.kYellow)) 
        );

        return armLigament;
    }
}
