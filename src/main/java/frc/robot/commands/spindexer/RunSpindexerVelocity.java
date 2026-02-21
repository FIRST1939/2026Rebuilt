package frc.robot.commands.spindexer;

import frc.robot.subsystems.spindexer.*;
import edu.wpi.first.wpilibj2.command.Command;

public class RunSpindexerVelocity extends Command {

    private final Spindexer m_spindexer;
    private final double m_velocity;

    public RunSpindexerVelocity(Spindexer spindexer, double velocity) {

        m_spindexer = spindexer;
        m_velocity = velocity;

        addRequirements(spindexer);
    }

    @Override
    public void initialize() {

        m_spindexer.setSpindexerVelocity(m_velocity);
    }

    @Override
    public void end(boolean interrupted) {
        m_spindexer.setSpindexerVelocity(0.0);
    }
}