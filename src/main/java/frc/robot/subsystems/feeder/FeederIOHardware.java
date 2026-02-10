package frc.robot.subsystems.feeder;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;

public class FeederIOHardware implements FeederIO {
    
    protected final SparkFlex m_motor = new SparkFlex(FeederConstants.kFeederCAN, MotorType.kBrushless);
    protected final RelativeEncoder m_encoder = m_motor.getEncoder();
    protected final SparkClosedLoopController m_controller = m_motor.getClosedLoopController(); 

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

    @Override 
    public void setFeederVelocity (double velocity) {
        
        m_controller.setSetpoint(velocity, ControlType.kVelocity, ClosedLoopSlot.kSlot0, 0.0);
    }
}
