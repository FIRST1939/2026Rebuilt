package frc.robot.commands.shooter;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.shooter.Shooter;

public class RunShooter extends Command {
    
    private final Shooter m_shooter;
    private final double m_percentage;

    public RunShooter (Shooter shooter, double percentage) {

        m_shooter = shooter;
        m_percentage = percentage;

        addRequirements(shooter);
    }

    @Override
    public void initialize () {

        m_shooter.setFlywheelPercentage(m_percentage);
    }

    @Override
    public void end (boolean interrupted) {

        m_shooter.setFlywheelPercentage(0.0);
    }
}
