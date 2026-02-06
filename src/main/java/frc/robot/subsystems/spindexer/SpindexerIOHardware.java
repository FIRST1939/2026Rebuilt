package frc.robot.subsystems.spindexer;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;

public class SpindexerIOHardware implements SpindexerIO {

    protected final SparkFlex m_motor = new SparkFlex(SpindexerConstants.kSpindexerCAN, MotorType.kBrushless);
    protected final SparkClosedLoopController m_controller = m_motor.getClosedLoopController(); 
    protected final RelativeEncoder m_encoder = m_motor.getEncoder(); 

    

    @Override
    public void updateInputs (SpindexerIOInputs inputs) {

        inputs.spindexerCurrent = m_motor.getOutputCurrent();
        inputs.spindexerVelocity = m_encoder.getVelocity();
        inputs.spindexerPosition = m_encoder.getPosition();
        inputs.spindexerVoltage = m_motor.getAppliedOutput() * m_motor.getBusVoltage();
        inputs.spindexerTemperature = m_motor.getMotorTemperature();
        }

    @Override 
    public void setSpindexerPercentage (double percent) {
            m_motor.set(percent);
    }

    @Override 
    public void setSpindexerVelocity (double velocity) {
        // Use the PID gains in slot 0 for position control
        // TODO make a constant?
        m_controller.setSetpoint(velocity, ControlType.kVelocity, ClosedLoopSlot.kSlot0);
    }
    
    
}
