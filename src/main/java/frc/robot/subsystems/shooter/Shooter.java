package frc.robot.subsystems.shooter;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.climber.ClimberIO;
import frc.robot.subsystems.climber.ClimberIOInputsAutoLogged;


public class Shooter extends SubsystemBase {

    private final ShooterIO m_io;
    private final ShooterIOInputsAutoLogged m_inputs = new ShooterIOInputsAutoLogged();

    public Shooter (ShooterIO io) {
        m_io = io;
    }

    public void periodic () {

        m_io.updateInputs(m_inputs);
        Logger.processInputs("Shooter", m_inputs);
    }

    public void setFlywheelPercentage (double percent) {
    }

    public void setFlywheelVelocity (double percent) {

    }

    public void getFlywheelVelocity () {

    }

    public void setHoodPercentage (double percent) {
        
    }

    public void setHoodPosition (double percent) {

    }

    public void getHoodPosition () {

    }
    
}
