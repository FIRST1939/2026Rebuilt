package frc.robot.subsystems.climber;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;

public class ClimberIOHardware implements ClimberIO {
    
    protected final SparkFlex m_motor = new SparkFlex(ClimberConstants.kClimberCAN,MotorType.kBrushless);
    protected final RelativeEncoder m_encoder = m_motor.getEncoder();

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
}
