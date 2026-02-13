package frc.robot.commands.shooter;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.shooter.Shooter;

public class RunShooter extends Command {
    
    private final Shooter shooter;
    private final double m_percentage;

    public RunShooter (Shooter shooter, double percentage) {

        this.shooter = shooter;
        m_percentage = percentage;
        this.addRequirements(this.shooter);
    }

    @Override
    public void initialize () {

        this.shooter.setFlywheelPercentage(m_percentage);
    }

    @Override
    public void end (boolean interrupted) {

        this.shooter.setFlywheelPercentage(0.0);
    }
}