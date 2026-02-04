package frc.robot.subsystems.spindexer;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Spindexer extends SubsystemBase {

    private final SpindexerIO m_io;
    private final SpindexerIOInputsAutoLogged m_inputs = new SpindexerIOInputsAutoLogged();


    public Spindexer (SpindexerIO io) {

        m_io = io;
    }
 
    @Override
    public void periodic() {

        m_io.updateInputs(m_inputs);
        Logger.processInputs("Spindexer", m_inputs);

    }

    public void setSpindexerPercentage (double percentage) {
        this.m_io.setSpindexerPercentage(percentage);
    }

    public void setSpindexerVelocity (double velocity) {
        this.m_io.setSpindexerVelocity(velocity);
    }
    
    public double getSpindexerVelocity () {
        return this.m_inputs.spindexerVelocity;
    }
}