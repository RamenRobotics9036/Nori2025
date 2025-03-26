package frc.robot.subsystems; 
 
import com.revrobotics.RelativeEncoder; 
import com.revrobotics.spark.SparkBase; 
import com.revrobotics.spark.SparkClosedLoopController; 
import com.revrobotics.spark.SparkLowLevel.MotorType; 
import com.revrobotics.spark.config.ClosedLoopConfig; 
import com.revrobotics.spark.config.EncoderConfig; 
import com.revrobotics.spark.config.SparkBaseConfig; 
import com.revrobotics.spark.config.SparkMaxConfig; 
import com.revrobotics.spark.SparkMax; 
import com.revrobotics.spark.SparkBase.ControlType; 
 
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard; 
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.simulation.DutyCycleEncoderSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase; 
 
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.SetArmToAngleCommand;
import frc.robot.sim.armsimulation.ArmDisplay;
import frc.robot.sim.armsimulation.ArmSimulation;
import frc.robot.sim.armsimulation.IOArmSim;
import frc.robot.sim.armsimulation.IOArmSimInterface;
import frc.robot.sim.simutil.RangeConvert;
import frc.robot.sim.simutil.RelativeEncoderSim;
import frc.robot.Constants.ArmConstants;
import frc.robot.logging.TriviaLogger;
 
 
public class IntakeArmSystem extends SubsystemBase{ 
    private SparkMax m_armMotor = new SparkMax(ArmConstants.kArmMotorID, MotorType.kBrushless); 
    private RelativeEncoder m_armRelativeEncoder = m_armMotor.getEncoder(); 
    private SparkMaxConfig m_armConfig = new SparkMaxConfig(); 
    private double maxOutput = ArmConstants.maxOutput; 
    private SparkClosedLoopController m_armPIDController = m_armMotor.getClosedLoopController(); 
    private final DutyCycleEncoder m_armEncoder = new DutyCycleEncoder(ArmConstants.kArmEncoderID); 
    private double desiredAngle; 
 
    private ArmSimulation m_armSimulation = null; 
    private ArmDisplay m_armDisplay = null;
    private RangeConvert m_rangesPhysicalAndSim = null; 
 
    //sets the idle mode of both motors to kBrake and adds a smartCurrentLimit 
    public IntakeArmSystem(){ 
        m_armEncoder.setInverted(true); 
        m_armEncoder.setDutyCycleRange(0, 1); 
 
        ClosedLoopConfig closedLoopConfig = new ClosedLoopConfig(); 
        closedLoopConfig 
            .p(1) 
            .i(0) 
            .d(0); 
        // closedLoopConfig.positionWrappingEnabled(true); 
        // closedLoopConfig.positionWrappingMinInput(0); 
        // closedLoopConfig.positionWrappingMaxInput(Math.PI * 2); 
 
        EncoderConfig encoderConfig = new EncoderConfig(); 
        encoderConfig.positionConversionFactor((Math.PI * 2) / ArmConstants.kArmGearBoxRatio); 
        encoderConfig.velocityConversionFactor(((Math.PI * 2) / ArmConstants.kArmGearBoxRatio) / 60); 
        // Did not set distance per rotation 
 
        // m_armConfig.inverted(true); 
 
        m_armConfig.idleMode(SparkBaseConfig.IdleMode.kCoast); 
        //m_armConfig.idleMode(SparkBaseConfig.IdleMode.kBrake); 
        m_armConfig.smartCurrentLimit(ArmConstants.kcurrentLimit); 
 
        m_armConfig.apply(closedLoopConfig); 
        m_armConfig.apply(encoderConfig); 
 
        m_armMotor.configure(m_armConfig,  
            SparkBase.ResetMode.kResetSafeParameters,  
            SparkBase.PersistMode.kPersistParameters); 

        // Configure how the arm will be displayed in AdvantageScope visually.
        // We map the physical min and max arm angles to -45 and 45 degrees in the visualization.
        m_rangesPhysicalAndSim = new RangeConvert( 
            Units.radiansToDegrees(ArmConstants.kMinArmRotation), 
            Units.radiansToDegrees(ArmConstants.kMaxArmRotation), 
            -45.0, 
            45.0,
            RobotBase.isSimulation() ? 5.0 : 15.0,
            true);

        if (RobotBase.isSimulation()) {         
            m_armSimulation = createSim(m_rangesPhysicalAndSim); 
        }

        if (!m_armEncoder.isConnected()) { 
            m_armRelativeEncoder.setPosition(0.0); 
        } else { 
            m_armRelativeEncoder.setPosition(getArmAngle()); 
        } 
 
        // if (!m_armEncoder.isConnected()) { 
        //     throw new ValueOutOfRangeException("ARM ABSOLUTE ENCODER NOT PLUGGED IN!", m_armEncoder.get()); 
        // }
        desiredAngle = getArmAngleRelative(); 
 
        initShuffleboard(); 
    } 
 
// private int loop = 0; 
 
    private ArmSimulation createSim(RangeConvert rangesPhysicalAndSim) { 
        // Create sim wrappers for devices 
        DutyCycleEncoderSim absEncoderSim = new DutyCycleEncoderSim(m_armEncoder); 
        RelativeEncoderSim relEncoderSim = new RelativeEncoderSim(m_armRelativeEncoder); 
 
        IOArmSimInterface ioArmSim = new IOArmSim( 
            absEncoderSim, 
            relEncoderSim, 
            () -> Units.radiansToDegrees(desiredAngle)); 
 
        return new ArmSimulation( 
            ioArmSim, 
            rangesPhysicalAndSim); 
    } 

    @Override 
    public void periodic() { 
        // loop += 1; 
        // if (loop % 50 == 0) { 
        //     System.out.println("@@@@ Arm encoder=" + getArmAngleRelative() + ", desired="+desiredAngle); 
        // } 
        if (m_armEncoder.isConnected()) { 
            m_armRelativeEncoder.setPosition(getArmAngle()); 
        }

        // Update the arm visualization
        m_armDisplay.setAngle(getArmAngle()); 
    } 
 
    @Override 
    public void simulationPeriodic() { 
        if (m_armSimulation != null) {
            m_armSimulation.simulationPeriodic(); 
        } 
    }

    public void initShuffleboard() { 
        if (!OperatorConstants.kCompetitionMode) { 
            ShuffleboardTab tab = Shuffleboard.getTab("Arm"); 
            tab.addDouble("Arm Relative Encoder", () -> getArmAngleRelative()); 
            tab.addDouble("Arm Encoder", () -> getArmAngle()); 
            tab.addDouble("Desired Angle", () -> desiredAngle); 
            tab.addBoolean("Encoder Is Connected", () -> m_armEncoder.isConnected()); 

            if (RobotBase.isSimulation()) { 
 
                // Add a button to test moving arm up 
                Command goUp = new SetArmToAngleCommand(this, ArmConstants.kMinArmRotation); 
                tab.add("Go Up2", goUp).withWidget("Command"); 
 
                Command goDown = new SetArmToAngleCommand(this, ArmConstants.kMaxArmRotation); 
                tab.add("Go Down2", goDown).withWidget("Command"); 
 
                Command goL1 = new SetArmToAngleCommand(this, ArmConstants.L1ArmAngle); 
                tab.add("Go L1", goL1).withWidget("Command"); 
            } 
        }

        initLogging();
  } 
 
  public void initLogging() {
    m_armDisplay = new ArmDisplay(m_rangesPhysicalAndSim);
    SmartDashboard.putData("ArmMechanism", m_armDisplay.getMech2d());

    // Log current command on this subsystem.
    TriviaLogger logger = TriviaLogger.getInstance();
    logger.registerSubsystemCmdCallback(
      getSubsystem(),
      () -> (this.getCurrentCommand() == null) ? "None"
              : this.getCurrentCommand().getName());
  }

    public void setReference(double position) { 
        desiredAngle = position; 
        m_armPIDController.setReference(position, ControlType.kPosition); 
    } 
 
    //sets the speed of m_armMotor. Cannot exceed maxOutputPercentage 
    public void setArmMotorSpeed(double speed){ 
        speed = MathUtil.clamp(speed, -maxOutput, maxOutput); 
        m_armMotor.set(speed); 
    } 
 
    //gets the speed of m_armMotor 
    public double getArmMotorSpeed(){ 
        return m_armMotor.get(); 
    } 
 
 
 
    // get encoder value 
    private double getArmAngle() {
        if (RobotBase.isSimulation()) { 
            return m_armEncoder.get(); 
        }

        /// $TODO This seems like a bug.  Conversion factor was already configured on the absolute encoder, 
        // so units are already in radians.  Needs investigation.
        return Math.max(0, (m_armEncoder.get() * 2 * Math.PI) + ArmConstants.kAbsoluteEncoderOffset) % (Math.PI * 2); 
    } 
 
    public double getArmAngleRelative() { 
        return m_armRelativeEncoder.getPosition(); 
    } 
 
    // public double getAbsoluteArmAngle() { 
    //     return m_armEncoder.getAbsolutePosition(); 
    // } 
 
    // public void resetArmAngle() { 
    //     m_armEncoder.reset(); 
    // } 
 
    //stops everything 
    public void stopSystem(){ 
        m_armMotor.stopMotor(); 
    } 
} 
