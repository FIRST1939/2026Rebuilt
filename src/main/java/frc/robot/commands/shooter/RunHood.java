package frc.robot.commands.shooter;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.shooter.Shooter;

public class RunHood extends Command {
    
    private final Shooter shooter;
    private final double m_percentage;

    public RunHood (Shooter shooter, double percentage) {

        this.shooter = shooter;
        m_percentage = percentage;
        this.addRequirements(this.shooter);
    }

    @Override
    public void initialize () {

        this.shooter.setHoodPercentage(m_percentage);
   //    this.shooter.setHoodPercentage(.5);
    }

    @Override
    public void end (boolean interrupted) {

        this.shooter.setHoodPercentage(0);
    }
}