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

public class IntakePivotIOHardware implements IntakePivotIO {

    protected final SparkFlex m_leftPivotMotor = new SparkFlex(IntakeConstants.kLeftPivotCAN, MotorType.kBrushless);
    protected final SparkFlex m_rightPivotMotor = new SparkFlex(IntakeConstants.kRightPivotCAN, MotorType.kBrushless);
    protected final RelativeEncoder m_leftPivotEncoder = m_leftPivotMotor.getEncoder();
    protected final RelativeEncoder m_rightPivotEncoder = m_rightPivotMotor.getEncoder();
    protected final SparkClosedLoopController m_leftPivotController = m_leftPivotMotor.getClosedLoopController();
    protected final SparkClosedLoopController m_rightPivotController = m_rightPivotMotor.getClosedLoopController();

    public IntakePivotIOHardware () {

        SparkFlexConfig globalPivotConfig = new SparkFlexConfig();

        globalPivotConfig
            .idleMode(IdleMode.kCoast)
            .smartCurrentLimit(IntakeConstants.kPivotCurrentLimit)
            .voltageCompensation(12.0);

        globalPivotConfig.encoder
            .positionConversionFactor(IntakeConstants.kPivotGearing)
            .velocityConversionFactor(IntakeConstants.kPivotGearing);

        globalPivotConfig.closedLoop.maxMotion
            .cruiseVelocity(IntakeConstants.kPivotProfileCruiseVelocity)
            .maxAcceleration(IntakeConstants.kPivotProfileMaxAcceleration)
            .allowedProfileError(IntakeConstants.kPivotProfileAllowedError);

        SparkFlexConfig leftPivotConfig = new SparkFlexConfig();

        leftPivotConfig
            .apply(globalPivotConfig)
            .inverted(IntakeConstants.kLeftPivotInverted);

        leftPivotConfig.closedLoop
            .p(IntakeConstants.kLeftPivotFeedbackP)
            .d(IntakeConstants.kLeftPivotFeedbackD)
            .feedForward
                .kS(IntakeConstants.kLeftPivotFeedforwardS)
                .kV(IntakeConstants.kLeftPivotFeedforwardV)
                .kA(IntakeConstants.kLeftPivotFeedforwardA);
        m_leftPivotMotor.configure(leftPivotConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        SparkFlexConfig rightPivotConfig = new SparkFlexConfig();

        rightPivotConfig
            .apply(globalPivotConfig)
            .inverted(IntakeConstants.kRightPivotInverted);

        rightPivotConfig.closedLoop
            .p(IntakeConstants.kRightPivotFeedbackP)
            .d(IntakeConstants.kRightPivotFeedbackD)
            .feedForward
                .kS(IntakeConstants.kRightPivotFeedforwardS)
                .kV(IntakeConstants.kRightPivotFeedforwardV)
                .kA(IntakeConstants.kRightPivotFeedforwardA);
        m_rightPivotMotor.configure(rightPivotConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    @Override
    public void updateInputs (IntakePivotIOInputs inputs) {

        inputs.leftPivotCurrent = m_leftPivotMotor.getOutputCurrent();
        inputs.leftPivotVelocity = m_leftPivotEncoder.getVelocity();
        inputs.leftPivotPosition = m_leftPivotEncoder.getPosition();
        inputs.leftPivotVoltage = m_leftPivotMotor.getAppliedOutput() * m_leftPivotMotor.getBusVoltage();
        inputs.leftPivotTemperature = m_leftPivotMotor.getMotorTemperature();

        inputs.rightPivotCurrent = m_rightPivotMotor.getOutputCurrent();
        inputs.rightPivotVelocity = m_rightPivotEncoder.getVelocity();
        inputs.rightPivotPosition = m_rightPivotEncoder.getPosition();
        inputs.rightPivotVoltage = m_rightPivotMotor.getAppliedOutput() * m_rightPivotMotor.getBusVoltage();
        inputs.rightPivotTemperature = m_rightPivotMotor.getMotorTemperature();
    }

    @Override
    public double getLeftPivotControllerPositionSetpoint () {

        return m_leftPivotController.getMAXMotionSetpointPosition();
    }

    @Override
    public double getLeftPivotControllerVelocitySetpoint () {

        return m_leftPivotController.getMAXMotionSetpointVelocity();
    }

    @Override
    public double getRightPivotControllerPositionSetpoint () {

        return m_leftPivotController.getMAXMotionSetpointPosition();
    }

    @Override
    public double getRightPivotControllerVelocitySetpoint () {

        return m_rightPivotController.getMAXMotionSetpointVelocity();
    }

    @Override
    public void updateLeftPivotControllerFeedback(double kP, double kD) {

        SparkFlexConfig config = new SparkFlexConfig();

        config.closedLoop
            .p(kP)
            .d(kD);

        m_leftPivotMotor.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
    }

    @Override
    public void updateRightPivotControllerFeedback(double kP, double kD) {

        SparkFlexConfig config = new SparkFlexConfig();

        config.closedLoop
            .p(kP)
            .d(kD);

        m_rightPivotMotor.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
    }

    @Override
    public void updatePivotControllerProfile (double kCruiseVelocity, double kMaxAcceleration, double kAllowedError) {

        SparkFlexConfig config = new SparkFlexConfig();

        config.closedLoop.maxMotion
            .cruiseVelocity(kCruiseVelocity)
            .maxAcceleration(kMaxAcceleration)
            .allowedProfileError(kAllowedError);

        m_leftPivotMotor.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
        m_rightPivotMotor.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
    }

    @Override
    public void setLeftPivotPercentage (double percent) {

        m_leftPivotMotor.set(percent);
    }

    @Override
    public void setLeftPivotVoltage (double voltage) {

        m_leftPivotController.setSetpoint(voltage, ControlType.kVoltage);
    }

    @Override
    public void setLeftPivotPosition (double position) {

        m_leftPivotController.setSetpoint(position, ControlType.kPosition);
    }

    @Override
    public void setRightPivotPercentage (double percent) {

        m_rightPivotMotor.set(percent);
    }

    @Override
    public void setRightPivotVoltage (double voltage) {

        m_rightPivotController.setSetpoint(voltage, ControlType.kVoltage);
    }

    @Override
    public void setRightPivotPosition (double position) {

        m_rightPivotController.setSetpoint(position, ControlType.kPosition);
    }

    @Override
    public boolean leftPivotIsAtSetpoint() {
        return m_leftPivotController.isAtSetpoint();
    }

    @Override
    public boolean rightPivotIsAtSetpoint() {
        return m_rightPivotController.isAtSetpoint();
    }
}
