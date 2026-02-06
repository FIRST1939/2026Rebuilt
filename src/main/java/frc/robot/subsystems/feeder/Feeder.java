package frc.robot.subsystems.feeder;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Feeder extends SubsystemBase {
    
    private final FeederIO m_io;
    private final FeederIOInputsAutoLogged m_inputs = new FeederIOInputsAutoLogged();

    public Feeder (FeederIO io) {
        
        m_io = io;
    }

    @Override

    public void periodic () {

        m_io.updateInputs(m_inputs);
        Logger.processInputs("Feeder", m_inputs);
    }
    public void setFeederPercentage (double percent) {
      
        m_io.setFeederPercentage(percent);

    }
    
    public void setFeederVelocity (double velocity) {

        m_io.setFeederVelocity (velocity);
    }

    public double getFeederVelocity () {
        
        return this.m_inputs.feederVelocity;
    }
}
