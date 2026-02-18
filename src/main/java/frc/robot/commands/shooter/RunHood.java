package frc.robot.commands.shooter;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.shooter.Shooter;

public class RunHood extends Command {
    
    private final Shooter m_hood;
    private final double m_percentage;

    public RunHood (Shooter hood, double percentage) {

        m_hood = hood;
        m_percentage = percentage;

        addRequirements(hood);
    }

    @Override
    public void initialize () {

        m_hood.setHoodPercentage(m_percentage);
    }

    @Override
    public void end (boolean interrupted) {

        m_hood.setHoodPercentage(0.0);
    }
}
