package frc.robot.subsystems.intake;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;

public class IntakeRollersIOHardware implements IntakeRollersIO {

    protected final SparkFlex m_rollerMotor = new SparkFlex(IntakeConstants.kRollerCAN, MotorType.kBrushless);
    protected final RelativeEncoder m_rollerEncoder = m_rollerMotor.getEncoder();
    protected final SparkClosedLoopController m_rollerController = m_rollerMotor.getClosedLoopController();

    public IntakeRollersIOHardware () {

        SparkFlexConfig rollerConfig = new SparkFlexConfig();

        rollerConfig
            .idleMode(IdleMode.kCoast)
            .inverted(IntakeConstants.kRollerInverted)
            .voltageCompensation(12.0);

        rollerConfig.encoder
            .positionConversionFactor(IntakeConstants.kRollerGearing)
            .velocityConversionFactor(IntakeConstants.kRollerGearing);

        rollerConfig.closedLoop
            .p(IntakeConstants.kRollerFeedbackP)
            .feedForward
                .kS(IntakeConstants.kRollerFeedforwardS)
                .kV(IntakeConstants.kRollerFeedforwardV)
                .kA(IntakeConstants.kRollerFeedforwardA);

        m_rollerMotor.configure(rollerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    @Override
    public void updateInputs (IntakeRollersIOInputs inputs) {

        inputs.rollerCurrent = m_rollerMotor.getOutputCurrent();
        inputs.rollerVelocity = m_rollerEncoder.getVelocity();
        inputs.rollerPosition = m_rollerEncoder.getPosition();
        inputs.rollerVoltage = m_rollerMotor.getAppliedOutput() * m_rollerMotor.getBusVoltage();
        inputs.rollerTemperature = m_rollerMotor.getMotorTemperature();
    }

    @Override
    public void updateRollerControllerFeedback (double kP, double kD) {

        SparkFlexConfig config = new SparkFlexConfig();

        config.closedLoop
            .p(kP)
            .d(kD);

        m_rollerMotor.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
    }

    @Override
    public void setRollerPercentage (double percent) {

        m_rollerMotor.set(percent);
    }

    @Override
    public void setRollerVoltage (double voltage) {

        m_rollerController.setSetpoint(voltage, ControlType.kVoltage);
    }

    @Override
    public void setRollerVelocity (double velocity) {

        m_rollerController.setSetpoint(velocity, ControlType.kVelocity);
    }
}
