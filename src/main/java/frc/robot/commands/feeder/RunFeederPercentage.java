package frc.robot.commands.feeder;

import frc.robot.subsystems.feeder.*;
import edu.wpi.first.wpilibj2.command.Command;

public class RunFeederPercentage extends Command {

    private final Feeder m_feeder;
    private final double m_percentage;

    public RunFeederPercentage(Feeder feeder, double percentage) {

        m_feeder = feeder;
        m_percentage = percentage;

        addRequirements(feeder);
    }

    @Override
    public void initialize() {

        m_feeder.setFeederPercentage(m_percentage);
    }

    @Override
    public void end(boolean interrupted) {
      
        m_feeder.setFeederPercentage(0);
    }
}
