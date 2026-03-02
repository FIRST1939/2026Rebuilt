package frc.robot.commands.climber;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.climber.Climber;

public class RaiseClimberToHeight extends Command {

    private final Climber m_climber;
    private final double m_height;
    private final double m_percentage;

    public RaiseClimberToHeight(Climber climber, double height, double percentage) {

        m_climber = climber;
        m_height = height;
        m_percentage = percentage;

        addRequirements(climber);
    }

    @Override
    public void initialize() {

        m_climber.setClimberPercentage(m_percentage);
    }

    
    @Override
    public boolean isFinished() {

        return m_climber.getClimberPosition() >= m_height;
    }


    @Override
    public void end (boolean interrupted) {
        
        m_climber.setClimberPercentage(0.0);
    }
}