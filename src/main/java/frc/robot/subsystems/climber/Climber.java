package frc.robot.subsystems.climber;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climber extends SubsystemBase {

    private final ClimberIO m_io;
    private final ClimberIOInputsAutoLogged m_inputs = new ClimberIOInputsAutoLogged();

    public Climber (ClimberIO io) {

        m_io = io;
    }

    @Override
    public void periodic () {

        m_io.updateInputs(m_inputs);
        Logger.processInputs("Climber", m_inputs);
    }

    public void setClimberPercentage (double percent) {
        
        m_io.setClimberPercentage(percent);
    }
    
    public void setClimberPosition() {
    }

}
