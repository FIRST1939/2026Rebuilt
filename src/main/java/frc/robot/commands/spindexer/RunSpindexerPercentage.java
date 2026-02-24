package frc.robot.commands.spindexer;

import frc.robot.subsystems.spindexer.*;
import edu.wpi.first.wpilibj2.command.Command;

public class RunSpindexerPercentage extends Command {

    private final Spindexer m_spindexer;
    private final double m_percentage;

    public RunSpindexerPercentage(Spindexer spindexer, double percentage) {

        m_spindexer = spindexer;
        m_percentage = percentage;

        addRequirements(spindexer);
    }

    @Override
    public void initialize() {

        m_spindexer.setSpindexerPercentage(m_percentage);
    }

     @Override
    public void end(boolean interrupted) {
       if (interrupted) {
            m_spindexer.setSpindexerPercentage(0.0);
        }
    }
}
