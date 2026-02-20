package frc.robot.subsystems.feeder;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;

import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;

public class FeederIOHardware implements FeederIO {
    
    protected final SparkFlex m_motor = new SparkFlex(FeederConstants.kFeederCAN, MotorType.kBrushless);
    protected final RelativeEncoder m_encoder = m_motor.getEncoder();
    protected final SparkClosedLoopController m_controller = m_motor.getClosedLoopController(); 

    public FeederIOHardware () {

        SparkFlexConfig config = new SparkFlexConfig();

        config
            .idleMode(IdleMode.kBrake)
            .inverted(FeederConstants.kInverted)
            .voltageCompensation(12.0);

        config.encoder
            .positionConversionFactor(FeederConstants.kFeederGearing)
            .velocityConversionFactor(FeederConstants.kFeederGearing);

        config.closedLoop
            .p(FeederConstants.kFeederFeedbackP)
            .feedForward
                .kS(FeederConstants.kFeederFeedforwardS)
                .kV(FeederConstants.kFeederFeedforwardV)
                .kA(FeederConstants.kFeederFeedforwardA);

        m_motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    @Override
    public void updateInputs (FeederIOInputs inputs) {

        inputs.feederPosition = m_encoder.getPosition();
        inputs.feederVelocity = m_encoder.getVelocity();
        inputs.feederVoltage = m_motor.getAppliedOutput() * m_motor.getBusVoltage();
        inputs.feederCurrent = m_motor.getOutputCurrent();
        inputs.feederTemperature = m_motor.getMotorTemperature();
    }

    @Override
    public void updateControllerFeedback (double kP, double kD) {

        SparkFlexConfig config = new SparkFlexConfig();

        config.closedLoop
            .p(kP)
            .d(kD);

        m_motor.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
    }

    @Override
    public void setFeederPercentage (double percentage) {

        m_motor.set(percentage);
    }

    @Override
    public void setFeederVoltage (double voltage) {

        m_controller.setSetpoint(voltage, ControlType.kVoltage);
    }

    @Override 
    public void setFeederVelocity (double velocity) {
        
        m_controller.setSetpoint(velocity, ControlType.kVelocity);
    }
}
