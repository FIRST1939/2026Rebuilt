package frc.robot.commands.shooter;

import java.util.function.DoubleSupplier;



import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.shooter.Shooter;


public class FollowShooterSetpoints extends Command {

    private final Shooter m_shooter;
    private final DoubleSupplier m_rpmSupplier;
    private final DoubleSupplier m_hoodPositionSupplier;
//This command follow the shooter setpoints.
//It doesn't park the hood when it ends. 

    public FollowShooterSetpoints(Shooter shooter, DoubleSupplier rpmSupplier, DoubleSupplier hoodPositionSupplier) {

        m_shooter = shooter;
        m_rpmSupplier = rpmSupplier;
        m_hoodPositionSupplier = hoodPositionSupplier;

        addRequirements(shooter);
    }

    @Override
    public void execute() {

        double targetPosition = m_hoodPositionSupplier.getAsDouble();

        m_shooter.setFlywheelVelocity(m_rpmSupplier.getAsDouble());
        m_shooter.setHoodPosition(targetPosition);
    }

    @Override
    public void end(boolean interrupted) {
        m_shooter.setFlywheelPercentage(0);
        m_shooter.setHoodPercentage(0);
    }
}
