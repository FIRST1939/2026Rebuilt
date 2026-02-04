package frc.robot.subsystems.shooter;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class Shooter extends SubsystemBase {

    private final ShooterIO m_io;
    private final ShooterIOInputsAutoLogged m_inputs = new ShooterIOInputsAutoLogged();

    public Shooter (ShooterIO io) {
        m_io = io;
    }

    @Override    
    public void periodic () {

        m_io.updateInputs(m_inputs);
        Logger.processInputs("Shooter", m_inputs);
    }

    public void setFlywheelPercentage (double percent) {
        m_io.setFlywheelPercentage(percent);
    }

    public void setFlywheelVelocity (double percent) {
        m_io.setFlywheelVelocity(percent);
    }

    public void getFlywheelPercentage () {

    }


    public void setHoodPercentage (double percent) {
        m_io.setHoodPercentage(percent);
    }

    public void setHoodPosition (double percent) {
        m_io.setHoodPosition(percent);
    }

    public void getHoodPosition () {

    }
    
}
