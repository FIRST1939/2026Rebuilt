package frc.robot.subsystems.feeder;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkFlex;

public class FeederIOHardware implements FeederIO {
    
    protected final SparkFlex m_motor = new SparkFlex(0, null);
     protected final RelativeEncoder m_encoder = m_motor.getEncoder();

      @Override
    public void updateInputs (FeederIOInputs inputs) {

        inputs.feederPosition = m_encoder.getPosition();
        inputs.feederVelocity = m_encoder.getVelocity();
        inputs.feederVoltage = m_motor.getAppliedOutput() * m_motor.getBusVoltage();
        inputs.feederCurrent = m_motor.getOutputCurrent();
        inputs.feederTemperature = m_motor.getMotorTemperature();
    }

    @Override
    public void setFeederPercentage (double percent) {

        m_motor.set(percent);
    }
}


