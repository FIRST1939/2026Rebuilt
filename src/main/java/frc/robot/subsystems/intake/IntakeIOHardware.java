package frc.robot.subsystems.intake;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

public class IntakeIOHardware implements IntakeIO {
    
    protected final SparkFlex m_roller = new SparkFlex(IntakeConstants.kRollerCAN, MotorType.kBrushless);
    protected final SparkFlex m_pivotLeader = new SparkFlex(IntakeConstants.kPivotFollowerCAN, MotorType.kBrushless);
    protected final SparkFlex m_pivotFollower = new SparkFlex(IntakeConstants.kPivotLeaderCAN, MotorType.kBrushless);
    protected final RelativeEncoder m_rollerEncoder = m_roller.getEncoder();
    protected final RelativeEncoder m_pivotLeaderEncoder = m_pivotLeader.getEncoder();
    protected final RelativeEncoder m_pivotFollowerEncoder = m_pivotFollower.getEncoder();
    protected final SparkClosedLoopController m_pivotController = m_pivotLeader.getClosedLoopController();
    protected final SparkClosedLoopController m_rollerController = m_roller.getClosedLoopController();

    public IntakeIOHardware () {

        SparkFlexConfig config = new SparkFlexConfig();

        config
            .idleMode(IdleMode.kBrake)
            .smartCurrentLimit(IntakeConstants.kRollerCurrentLimit);

        config.encoder
        .positionConversionFactor(1.0 / IntakeConstants.kRollerGearReduction)
        .velocityConversionFactor(1.0 / IntakeConstants.kRollerGearReduction)
        .inverted(IntakeConstants.kInverted);

        m_roller.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        
    }

    @Override
    public void updateInputs (IntakeIOInputs inputs) {

        inputs.rollerCurrent = m_roller.getOutputCurrent();
        inputs.rollerVelocity = m_rollerEncoder.getVelocity();
        inputs.rollerPosition = m_rollerEncoder.getPosition();
        inputs.rollerVoltage = m_roller.getAppliedOutput() * m_roller.getBusVoltage();
        inputs.rollerTemperature = m_roller.getMotorTemperature();

        inputs.rollerCurrent = m_pivotLeader.getOutputCurrent();
        inputs.rollerVelocity = m_pivotLeaderEncoder.getVelocity();
        inputs.rollerPosition = m_pivotLeaderEncoder.getPosition();
        inputs.rollerVoltage = m_pivotLeader.getAppliedOutput() * m_pivotLeader.getBusVoltage();
        inputs.rollerTemperature = m_pivotLeader.getMotorTemperature();

        inputs.rollerCurrent = m_pivotFollower.getOutputCurrent();
        inputs.rollerVelocity = m_pivotFollowerEncoder.getVelocity();
        inputs.rollerPosition = m_pivotFollowerEncoder.getPosition();
        inputs.rollerVoltage = m_pivotFollower.getAppliedOutput() * m_pivotFollower.getBusVoltage();
        inputs.rollerTemperature = m_pivotFollower.getMotorTemperature();
    }

    @Override
    public void setPivotPosition (double position) {
        m_pivotController.setSetpoint(position, ControlType.kMAXMotionPositionControl, ClosedLoopSlot.kSlot0, 0.0);
    }

    @Override
    public void setRollerVelocity (double velocity) {
        m_rollerController.setSetpoint(velocity, ControlType.kVelocity, ClosedLoopSlot.kSlot0, 0.0);
    }
}
