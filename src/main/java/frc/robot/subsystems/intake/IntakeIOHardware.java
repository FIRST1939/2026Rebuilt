package frc.robot.subsystems.intake;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;

public class IntakeIOHardware implements IntakeIO {
    
    protected final SparkFlex m_roller = new SparkFlex(IntakeConstants.kRollerCAN, MotorType.kBrushless);
    protected final SparkFlex m_pivotLeader = new SparkFlex(IntakeConstants.kPivotFollowerCAN, MotorType.kBrushless);
    protected final SparkFlex m_pivotFollower = new SparkFlex(IntakeConstants.kPivotLeaderCAN, MotorType.kBrushless);
    protected final RelativeEncoder m_rollerEncoder = m_roller.getEncoder();
    protected final RelativeEncoder m_pivotLeaderEncoder = m_pivotLeader.getEncoder();
    protected final RelativeEncoder m_pivotFollowerEncoder = m_pivotFollower.getEncoder();

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
}
