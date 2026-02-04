package frc.robot.subsystems.shooter;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import frc.robot.subsystems.climber.ClimberIO.ClimberIOInputs;

public class ShooterIOHardware implements ShooterIO {
    
    protected final SparkFlex m_flywheelOne = new SparkFlex(ShooterConstants.kFlywheelOneCAN, MotorType.kBrushless);
    protected final RelativeEncoder m_flywheelOneEncoder = m_flywheelOne.getEncoder();

    protected final SparkFlex m_flywheelTwo = new SparkFlex(ShooterConstants.kFlywheelTwoCAN, MotorType.kBrushless);
    protected final RelativeEncoder m_flywheelTwoEncoder = m_flywheelTwo.getEncoder();

    protected final SparkFlex m_hood = new SparkFlex(ShooterConstants.kHoodCAN, MotorType.kBrushless);
    protected final RelativeEncoder m_hoodEncoder = m_hood.getEncoder();


    @Override
    public void updateInputs (ShooterIOInputs inputs) {

        inputs.flywheelOnePosition = m_flywheelOneEncoder.getPosition();
        inputs.flyWheelOneVelocity = m_flywheelOneEncoder.getVelocity();
        inputs.flywheelOneVoltage = m_flywheelOne.getAppliedOutput() * m_flywheelOne.getBusVoltage();
        inputs.flywheelOneCurrent = m_flywheelOne.getOutputCurrent();
        inputs.flywheelOneTemperature = m_flywheelOne.getMotorTemperature();

        inputs.flywheelTwoPosition = m_flywheelTwoEncoder.getPosition();
        inputs.flyWheelTwoVelocity = m_flywheelTwoEncoder.getVelocity();
        inputs.flywheelTwoVoltage = m_flywheelTwo.getAppliedOutput() * m_flywheelTwo.getBusVoltage();
        inputs.flywheelTwoCurrent = m_flywheelTwo.getOutputCurrent();
        inputs.flywheelTwoTemperature = m_flywheelTwo.getMotorTemperature();

        inputs.hoodPosition = m_hoodEncoder.getPosition();
        inputs.hoodVelocity = m_hoodEncoder.getVelocity();
        inputs.hoodVoltage = m_hood.getAppliedOutput() * m_hood.getBusVoltage();
        inputs.hoodCurrent = m_hood.getOutputCurrent();
        inputs.hoodTemperature = m_hood.getMotorTemperature();
    }

    @Override
    public void setFlywheelPercentage (double percent) {

        m_flywheelOne.set(percent);
        m_flywheelTwo.set(percent * -1);
    }
}