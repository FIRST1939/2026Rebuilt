package frc.robot.subsystems.climber;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;

public class ClimberIOHardware implements ClimberIO {
    
    protected final SparkFlex m_motor = new SparkFlex(ClimberConstants.kClimberCAN, MotorType.kBrushless);
    protected final SparkClosedLoopController m_controller = m_motor.getClosedLoopController();
    protected final RelativeEncoder m_encoder = m_motor.getEncoder();

    public ClimberIOHardware () {

        SparkFlexConfig config = new SparkFlexConfig();

        config
            .idleMode(IdleMode.kBrake)
            .inverted(ClimberConstants.kInverted)
            .smartCurrentLimit(ClimberConstants.kCurrentLimit);

        config.encoder
            .positionConversionFactor(1.0 / ClimberConstants.kClimberGearReduction)
            .velocityConversionFactor(1.0 / ClimberConstants.kClimberGearReduction);

        m_motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    @Override
    public void updateInputs (ClimberIOInputs inputs) {

        inputs.climberPosition = m_encoder.getPosition();
        inputs.climberVelocity = m_encoder.getVelocity();
        inputs.climberVoltage = m_motor.getAppliedOutput() * m_motor.getBusVoltage();
        inputs.climberCurrent = m_motor.getOutputCurrent();
        inputs.climberTemperature = m_motor.getMotorTemperature();
    }

    @Override
    public void setClimberPercentage (double percent) {

        m_motor.set(percent);
    }

    @Override
    public void setClimberPosition (double position) {

        m_controller.setSetpoint(position, ControlType.kMAXMotionPositionControl, ClosedLoopSlot.kSlot0, 0.0);
    }
}
