package frc.robot.commands.feeder;

import frc.robot.subsystems.feeder.*;
import edu.wpi.first.wpilibj2.command.Command;

public class RunFeederVelocity extends Command {

    private final Feeder m_feeder;
    private final double m_velocity;

    public RunFeederVelocity(Feeder feeder, double velocity) {

        m_feeder = feeder;
        m_velocity = velocity;

        addRequirements(feeder);
    }

    @Override
    public void initialize() {

        m_feeder.setFeederVelocity(m_velocity);
    }

    @Override
    public void end (boolean interrupted) {
       if (interrupted) {
            m_feeder.setFeederPercentage(0.0);
       }
    }
}