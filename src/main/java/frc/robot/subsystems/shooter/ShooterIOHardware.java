package frc.robot.subsystems.shooter;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;

public class ShooterIOHardware implements ShooterIO {
    
    protected final SparkFlex m_flywheelLeader = new SparkFlex(ShooterConstants.kFlywheelLeaderCAN, MotorType.kBrushless);
    protected final RelativeEncoder m_flywheelLeaderEncoder = m_flywheelLeader.getEncoder();

    protected final SparkFlex m_flywheelFollower = new SparkFlex(ShooterConstants.kFlywheelFollowerCAN, MotorType.kBrushless);
    protected final RelativeEncoder m_flywheelFollowerEncoder = m_flywheelFollower.getEncoder();

    protected final SparkFlex m_hood = new SparkFlex(ShooterConstants.kHoodCAN, MotorType.kBrushless);
    protected final RelativeEncoder m_hoodEncoder = m_hood.getEncoder();


    @Override
    public void updateInputs (ShooterIOInputs inputs) {

        inputs.flywheelOnePosition = m_flywheelLeaderEncoder.getPosition();
        inputs.flyWheelOneVelocity = m_flywheelLeaderEncoder.getVelocity();
        inputs.flywheelOneVoltage = m_flywheelLeader.getAppliedOutput() * m_flywheelLeader.getBusVoltage();
        inputs.flywheelOneCurrent = m_flywheelLeader.getOutputCurrent();
        inputs.flywheelOneTemperature = m_flywheelLeader.getMotorTemperature();

        inputs.flywheelTwoPosition = m_flywheelFollowerEncoder.getPosition();
        inputs.flyWheelTwoVelocity = m_flywheelFollowerEncoder.getVelocity();
        inputs.flywheelTwoVoltage = m_flywheelFollower.getAppliedOutput() * m_flywheelFollower.getBusVoltage();
        inputs.flywheelTwoCurrent = m_flywheelFollower.getOutputCurrent();
        inputs.flywheelTwoTemperature = m_flywheelFollower.getMotorTemperature();

        inputs.hoodPosition = m_hoodEncoder.getPosition();
        inputs.hoodVelocity = m_hoodEncoder.getVelocity();
        inputs.hoodVoltage = m_hood.getAppliedOutput() * m_hood.getBusVoltage();
        inputs.hoodCurrent = m_hood.getOutputCurrent();
        inputs.hoodTemperature = m_hood.getMotorTemperature();
    }

    @Override
    public void setFlywheelPercentage (double percent) {

        m_flywheelLeader.set(percent);
        m_flywheelFollower.set(percent * -1);
    }
}