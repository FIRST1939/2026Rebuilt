package frc.robot.commands.feeder;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.feeder.Feeder;


public class StartFeederVelocity extends Command {

    private final Feeder m_feeder;
    private final double m_velocity;
    private final double m_velocityThreshold;

    public StartFeederVelocity(Feeder feeder, double velocity, double velocityThreshold) {
        m_feeder = feeder;
        m_velocity = velocity;
        m_velocityThreshold = velocityThreshold;

        addRequirements(feeder);
    }

    @Override
    public void initialize() {
        m_feeder.setFeederVelocity(m_velocity);
    }

    @Override
    public boolean isFinished() {
        return Math.abs(m_feeder.getFeederVelocity()) > m_velocityThreshold;
    }
}
