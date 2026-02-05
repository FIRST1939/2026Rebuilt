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

        inputs.flywheelLeaderPosition = m_flywheelLeaderEncoder.getPosition();
        inputs.flywheelLeaderVelocity = m_flywheelLeaderEncoder.getVelocity();
        inputs.flywheelLeaderVoltage = m_flywheelLeader.getAppliedOutput() * m_flywheelLeader.getBusVoltage();
        inputs.flywheelLeaderCurrent = m_flywheelLeader.getOutputCurrent();
        inputs.flywheelLeaderTemperature = m_flywheelLeader.getMotorTemperature();

        inputs.flywheelFollowerPosition = m_flywheelFollowerEncoder.getPosition();
        inputs.flywheelFollowerVelocity = m_flywheelFollowerEncoder.getVelocity();
        inputs.flywheelFollowerVoltage = m_flywheelFollower.getAppliedOutput() * m_flywheelFollower.getBusVoltage();
        inputs.flywheelFollowerCurrent = m_flywheelFollower.getOutputCurrent();
        inputs.flywheelFollowerTemperature = m_flywheelFollower.getMotorTemperature();

        inputs.hoodPosition = m_hoodEncoder.getPosition();
        inputs.hoodVelocity = m_hoodEncoder.getVelocity();
        inputs.hoodVoltage = m_hood.getAppliedOutput() * m_hood.getBusVoltage();
        inputs.hoodCurrent = m_hood.getOutputCurrent();
        inputs.hoodTemperature = m_hood.getMotorTemperature();
    }
}