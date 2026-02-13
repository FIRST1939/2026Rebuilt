package frc.robot.subsystems.spindexer;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkClosedLoopController;

public class SpindexerIOHardware implements SpindexerIO {

    protected final SparkFlex m_motor = new SparkFlex(SpindexerConstants.kSpindexerCAN, MotorType.kBrushless);
    protected final RelativeEncoder m_encoder = m_motor.getEncoder(); 
    protected final SparkClosedLoopController m_spindexerController = m_motor.getClosedLoopController();

    public SpindexerIOHardware() {

        SparkFlexConfig config = new SparkFlexConfig();

        config
            .idleMode(IdleMode.kCoast)
            .smartCurrentLimit(SpindexerConstants.kCurrentLimit);

        config.encoder
            .velocityConversionFactor(SpindexerConstants.kSpindexerGearing)
            .positionConversionFactor(SpindexerConstants.kSpindexerGearing);
           //inverted(SpindexerConstants.kInverted);

        // config
        // .closedLoop
        //       .pid(0, 0, 0)
            
        //     .feedForward
        //         .kS(SpindexerConstants.kSpindexerFeedforwardS, ClosedLoopSlot.kSlot0)
        //         .kV(SpindexerConstants.kSpindexerFeedforwardV, ClosedLoopSlot.kSlot0)
        //         .kA(SpindexerConstants.kSpindexerFeedforwardA, ClosedLoopSlot.kSlot0);


        m_motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    }

    @Override
    public void updateInputs (SpindexerIOInputs inputs) {

        inputs.spindexerCurrent = m_motor.getOutputCurrent();
        inputs.spindexerVelocity = m_encoder.getVelocity();
        inputs.spindexerPosition = m_encoder.getPosition();
        inputs.spindexerVoltage = m_motor.getAppliedOutput() * m_motor.getBusVoltage();
        inputs.spindexerTemperature = m_motor.getMotorTemperature();
    }

    public void setSpindexerPercentage(double percentage) {
        
        m_motor.set(percentage);
    }

    @Override
    public void setSpindexerVoltage (double voltage) {

        m_spindexerController.setSetpoint(voltage, ControlType.kVoltage);
    }

    @Override
    public void setSpindexerVelocity (double velocity) {
        
        m_spindexerController.setSetpoint(velocity, ControlType.kVelocity, ClosedLoopSlot.kSlot0);
    }
}
