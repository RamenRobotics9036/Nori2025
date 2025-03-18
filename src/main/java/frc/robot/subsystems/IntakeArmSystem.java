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
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.ArmConstants;
import frc.robot.commands.SetArmToAngleCommand;
import frc.robot.subsystems.armsimulation.ArmSimulation;
import frc.robot.subsystems.armsimulation.IOArmSim;
import frc.robot.subsystems.armsimulation.IOArmSimInterface;
import frc.robot.util.RelativeEncoderSim;
import edu.wpi.first.wpilibj.simulation.DutyCycleEncoderSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.RobotBase;

public class IntakeArmSystem extends SubsystemBase{
    private SparkMax m_armMotor = new SparkMax(ArmConstants.kArmMotorID, MotorType.kBrushless);
    private RelativeEncoder m_armRelativeEncoder = m_armMotor.getEncoder();
    private SparkMaxConfig m_armConfig = new SparkMaxConfig();
    private double maxOutput = ArmConstants.maxOutput;
    private SparkClosedLoopController m_armPIDController = m_armMotor.getClosedLoopController();
    private final DutyCycleEncoder m_armEncoder = new DutyCycleEncoder(ArmConstants.kArmEncoderID);
    private double desiredAngleRads;

    private ArmSimulation m_armSimulation = null;

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

        if (!m_armEncoder.isConnected()) {
            m_armRelativeEncoder.setPosition(0.0);
        } else {
            m_armRelativeEncoder.setPosition(getArmAngleRads());
        }

        // if (!m_armEncoder.isConnected()) {
        //     throw new ValueOutOfRangeException("ARM ABSOLUTE ENCODER NOT PLUGGED IN!", m_armEncoder.get());
        // }
        desiredAngleRads = getArmAngleRelativeRads();

        if (RobotBase.isSimulation()) {
            m_armSimulation = createSim();
        }

        initShuffleboard();
    }

    private ArmSimulation createSim() {
        // Create sim wrappers for devices
        DutyCycleEncoderSim absEncoderSim = new DutyCycleEncoderSim(m_armEncoder);
        RelativeEncoderSim relEncoderSim = new RelativeEncoderSim(m_armRelativeEncoder);

        IOArmSimInterface ioArmSim = new IOArmSim(
            absEncoderSim,
            relEncoderSim,
            () -> 20); // $TODO Units.radiansToDegrees(desiredAngleRads));

        return new ArmSimulation(ioArmSim);
    }

// private int loop = 0;

    @Override
    public void periodic() {
        // loop += 1;
        // if (loop % 50 == 0) {
        //     System.out.println("@@@@ Arm encoder=" + getArmAngleRelative() + ", desired="+desiredAngle);
        // }
        if (m_armEncoder.isConnected()) {
            m_armRelativeEncoder.setPosition(getArmAngleRads());
        }
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
            tab.addDouble("Arm Relative Encoder Rads", () -> getArmAngleRelativeRads());
            tab.addDouble("Arm Encoder Rads", () -> getArmAngleRads());
            tab.addDouble("Desired Angle Rads", () -> desiredAngleRads);
            tab.addBoolean("Encoder Is Connected", () -> m_armEncoder.isConnected());

            // Show current command on shuffleboard
            tab.addString(
            "IntakeArmSystem Command",
            () -> (this.getCurrentCommand() == null) ? "None"
                    : this.getCurrentCommand().getName());

            if (m_armSimulation != null) {
                SmartDashboard.putData("Arm Sim", m_armSimulation.getMech2d());

                // Add a button to test moving arm up
                Command goUp = new SetArmToAngleCommand(this, ArmConstants.kMinArmRotation);
                tab.add("Go Up2", goUp).withWidget("Command");
            }
        }
  }

    public void setReferenceRads(double newReferencePositionRads) {
        desiredAngleRads = newReferencePositionRads;
        m_armPIDController.setReference(newReferencePositionRads, ControlType.kPosition);
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
    private double getArmAngleRads() {
        // $TODO - Is there a bug here?  Isnt m_armEncoder.get() already returning in radians since
        // we configured the encoders to use a conversion factor above?
        return Math.max(0, (m_armEncoder.get() * 2 * Math.PI) + ArmConstants.kAbsoluteEncoderOffset) % (Math.PI * 2);
    }

    public double getArmAngleRelativeRads() {
        return m_armRelativeEncoder.getPosition();
    }

    //stops everything
    public void stopSystem(){
        m_armMotor.stopMotor();
    }
}
