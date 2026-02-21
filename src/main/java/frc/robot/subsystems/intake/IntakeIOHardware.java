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

public class IntakeIOHardware implements IntakeIO {
    
    protected final SparkFlex m_rollerMotor = new SparkFlex(IntakeConstants.kRollerCAN, MotorType.kBrushless);
    protected final SparkFlex m_leftPivotMotor = new SparkFlex(IntakeConstants.kLeftPivotCAN, MotorType.kBrushless);
    protected final SparkFlex m_rightPivotMotor = new SparkFlex(IntakeConstants.kRightPivotCAN, MotorType.kBrushless);
    protected final RelativeEncoder m_rollerEncoder = m_rollerMotor.getEncoder();
    protected final RelativeEncoder m_leftPivotEncoder = m_leftPivotMotor.getEncoder();
    protected final RelativeEncoder m_rightPivotEncoder = m_rightPivotMotor.getEncoder();
    protected final SparkClosedLoopController m_rollerController = m_rollerMotor.getClosedLoopController();
    protected final SparkClosedLoopController m_leftPivotController = m_leftPivotMotor.getClosedLoopController();
    protected final SparkClosedLoopController m_rightPivotController = m_rightPivotMotor.getClosedLoopController();

    public IntakeIOHardware () {

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
            .inverted(IntakeConstants.kLeftPivotInverted)
            .idleMode(IdleMode.kCoast);

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
            .inverted(IntakeConstants.kRightPivotInverted)
            .idleMode(IdleMode.kBrake);

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
    public void updateInputs (IntakeIOInputs inputs) {

        inputs.rollerCurrent = m_rollerMotor.getOutputCurrent();
        inputs.rollerVelocity = m_rollerEncoder.getVelocity();
        inputs.rollerPosition = m_rollerEncoder.getPosition();
        inputs.rollerVoltage = m_rollerMotor.getAppliedOutput() * m_rollerMotor.getBusVoltage();
        inputs.rollerTemperature = m_rollerMotor.getMotorTemperature();

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
    public void updateRollerControllerFeedback (double kP, double kD) {

        SparkFlexConfig config = new SparkFlexConfig();

        config.closedLoop
            .p(kP)
            .d(kD);

        m_rollerMotor.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
    }

    @Override
    public double getLeftPivotControllerSetpoint () {

        return m_leftPivotController.getMAXMotionSetpointPosition();
    }

    @Override
    public double getRightPivotControllerSetpoint () {

        return m_leftPivotController.getMAXMotionSetpointPosition();
    }

    @Override
    public void updatePivotControllerFeedback (double kP, double kD) {

        SparkFlexConfig config = new SparkFlexConfig();

        config.closedLoop
            .p(kP)
            .d(kD);

        m_leftPivotMotor.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
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

        m_leftPivotController.setSetpoint(position, ControlType.kMAXMotionPositionControl);
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

        m_rightPivotController.setSetpoint(position, ControlType.kMAXMotionPositionControl);
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
