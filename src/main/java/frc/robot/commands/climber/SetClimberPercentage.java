package frc.robot.commands.climber;

import static edu.wpi.first.units.Units.Percent;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.climber.Climber;

public class SetClimberPercentage extends Command {

    private final Climber m_climber;
    private final double m_percentage;

    public SetClimberPercentage(Climber climber, double percentage) {
        m_climber = climber;
        m_percentage = percentage;

        addRequirements(climber);
    }

    @Override
    public void initialize() {
        m_climber.setClimberPercentage(m_percentage);
    }


    @Override
    public void end (boolean interrupted) {
       if (interrupted) {
            m_climber.setClimberPercentage(0.0);
        }
    }
}