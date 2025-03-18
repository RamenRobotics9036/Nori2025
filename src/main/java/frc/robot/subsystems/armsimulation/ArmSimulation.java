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
        // Read the setpoint from the IO interface
        double desiredAngle = m_ioArmSimInterface.getSetpoint();
        double currentAngle = m_armSim.getAngleRads();

        // Use PID controller to get motor voltage
        double simOutput = m_pidController.calculate(currentAngle, desiredAngle);
        double motorVolts = MathUtil.clamp(simOutput, -1.0, 1.0) * 12.0;

        // Run the simulation with a particular motor voltage
        m_armSim.setInput(motorVolts);
        m_armSim.update(0.02);

        // Set the output
        double newAngle = m_armSim.getAngleRads();
        double newAngleDegrees = Units.radiansToDegrees(newAngle);
        m_armLigament.setAngle(newAngleDegrees);

        // Write the new position into the absolute encoder
        m_ioArmSimInterface.setOutputArmAngleAbsolute(newAngle);
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
