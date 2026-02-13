package frc.robot.subsystems.spindexer;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkClosedLoopController;

public class SpindexerIOHardware implements SpindexerIO {

    protected final SparkFlex m_motor = new SparkFlex(SpindexerConstants.kSpindexerCAN, MotorType.kBrushless);
    protected final RelativeEncoder m_encoder = m_motor.getEncoder(); 
    protected final SparkClosedLoopController m_spindexerController = m_motor.getClosedLoopController();

    public SpindexerIOHardware() {

        SparkFlexConfig config = new SparkFlexConfig();

        config
            .idleMode(IdleMode.kBrake)
            .inverted(SpindexerConstants.kInverted)
            .smartCurrentLimit(SpindexerConstants.kCurrentLimit);

        config.encoder
            .velocityConversionFactor(SpindexerConstants.kSpindexerGearing)
            .positionConversionFactor(SpindexerConstants.kSpindexerGearing);

        m_motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    }

    public void setSpindexerPercentage(double percent) {
        
    m_spindexerController.setSetpoint(percent, ControlType.kVelocity, ClosedLoopSlot.kSlot0);
    //NOTE: ControlType.kVelocity here is supposed to be Feed Forward. We will just let PID stuff equal zero.
    }
    @Override
    public void updateInputs (SpindexerIOInputs inputs) {

        inputs.spindexerCurrent = m_motor.getOutputCurrent();
        inputs.spindexerVelocity = m_encoder.getVelocity();
        inputs.spindexerPosition = m_encoder.getPosition();
        inputs.spindexerVoltage = m_motor.getAppliedOutput() * m_motor.getBusVoltage();
        inputs.spindexerTemperature = m_motor.getMotorTemperature();
        
    }
}
