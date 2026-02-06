package frc.robot.subsystems.feeder;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Feeder extends SubsystemBase {

    private final FeederIO m_io;
    private final FeederIOInputsAutoLogged m_inputs = new FeederIOInputsAutoLogged();


    public Feeder (FeederIO io) {

        m_io = io;
    }
 
    @Override
    public void periodic() {

        m_io.updateInputs(m_inputs);
        Logger.processInputs("Feeder", m_inputs);

    }

    public void setFeederPercentage (double percentage) {
        this.m_io.setFeederPercentage(percentage);
    }

    public void setFeederVelocity (double velocity) {
        this.m_io.setFeederVelocity(velocity);
    }
    
    public double getFeederVelocity () {
      return 0.0;
      //todo fix this
       // return this.m_inputs.FeederVelocity;
    }
}