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
            //.smartCurrentLimit(ClimberConstants.kCurrentLimit)
            .voltageCompensation(12.0);

        config.encoder
            .positionConversionFactor(ClimberConstants.kClimberGearing)
            .velocityConversionFactor(ClimberConstants.kClimberGearing);

        config.closedLoop
            .p(ClimberConstants.kRaisingFeedbackP, ClosedLoopSlot.kSlot0)
            .d(ClimberConstants.kRaisingFeedbackD, ClosedLoopSlot.kSlot0)
            .feedForward
                .kS(ClimberConstants.kRaisingFeedforwardS, ClosedLoopSlot.kSlot0)
                .kV(ClimberConstants.kRaisingFeedforwardV, ClosedLoopSlot.kSlot0)
                .kA(ClimberConstants.kRaisingFeedforwardA, ClosedLoopSlot.kSlot0)
                .kG(ClimberConstants.kRaisingFeedforwardG, ClosedLoopSlot.kSlot0);

        config.closedLoop.maxMotion
            .cruiseVelocity(ClimberConstants.kProfileCruiseVelocity, ClosedLoopSlot.kSlot0)
            .maxAcceleration(ClimberConstants.kProfileMaxAcceleration, ClosedLoopSlot.kSlot0)
            .allowedProfileError(ClimberConstants.kProfileAllowedError, ClosedLoopSlot.kSlot0);

        config.closedLoop
            .p(ClimberConstants.kClimbingFeedbackP, ClosedLoopSlot.kSlot1)
            .d(ClimberConstants.kClimbingFeedbackD, ClosedLoopSlot.kSlot1)
            .feedForward
                .kS(ClimberConstants.kClimbingFeedforwardS, ClosedLoopSlot.kSlot1)
                .kV(ClimberConstants.kClimbingFeedforwardV, ClosedLoopSlot.kSlot1)
                .kA(ClimberConstants.kClimbingFeedforwardA, ClosedLoopSlot.kSlot1)
                .kG(ClimberConstants.kClimbingFeedforwardG, ClosedLoopSlot.kSlot1);

        config.closedLoop.maxMotion
            .cruiseVelocity(ClimberConstants.kProfileCruiseVelocity, ClosedLoopSlot.kSlot1)
            .maxAcceleration(ClimberConstants.kProfileMaxAcceleration, ClosedLoopSlot.kSlot1)
            .allowedProfileError(ClimberConstants.kProfileAllowedError, ClosedLoopSlot.kSlot1);

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
    public double getControllerSetpoint () {
        return m_controller.getMAXMotionSetpointPosition();
    }

    @Override
    public void updateRaisingControllerFeedback (double kP, double kD) {

        SparkFlexConfig config = new SparkFlexConfig();

        config.closedLoop
            .p(kP, ClosedLoopSlot.kSlot0)
            .d(kD, ClosedLoopSlot.kSlot0);

        m_motor.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
    }

    @Override
    public void updateClimbingControllerFeedback (double kP, double kD) {

        SparkFlexConfig config = new SparkFlexConfig();

        config.closedLoop
            .p(kP, ClosedLoopSlot.kSlot1)
            .d(kD, ClosedLoopSlot.kSlot1);

        m_motor.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
    }

    @Override
    public void updateControllerProfile (double kCruiseVelocity, double kMaxAcceleration, double kAllowedError) {

        SparkFlexConfig config = new SparkFlexConfig();

        config.closedLoop.maxMotion
            .cruiseVelocity(kCruiseVelocity, ClosedLoopSlot.kSlot0)
            .maxAcceleration(kMaxAcceleration, ClosedLoopSlot.kSlot0)
            .allowedProfileError(kAllowedError, ClosedLoopSlot.kSlot0);

        config.closedLoop.maxMotion
            .cruiseVelocity(kCruiseVelocity, ClosedLoopSlot.kSlot1)
            .maxAcceleration(kMaxAcceleration, ClosedLoopSlot.kSlot1)
            .allowedProfileError(kAllowedError, ClosedLoopSlot.kSlot1);

        m_motor.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
    }

    @Override
    public void setClimberPercentage (double percent) {

        m_motor.set(percent);
    }

    @Override
    public void setClimberVoltage (double voltage) {

        m_controller.setSetpoint(voltage, ControlType.kVoltage);
    }

    @Override
    public void setRaisingPosition (double position) {

        m_controller.setSetpoint(position, ControlType.kMAXMotionPositionControl, ClosedLoopSlot.kSlot0);
    }

    @Override
    public void setClimbingPosition (double position) {

        m_controller.setSetpoint(position, ControlType.kMAXMotionPositionControl, ClosedLoopSlot.kSlot1);
    }

    @Override
    public boolean isAtSetpoint() {
        return m_controller.isAtSetpoint();
    }
}
