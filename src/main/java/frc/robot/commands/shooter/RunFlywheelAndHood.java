package frc.robot.commands.shooter;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.shooter.Shooter;

public class RunFlywheelAndHood extends Command {

    private final Shooter m_shooter;
    private final DoubleSupplier m_flywheelVelocitySupplier;
    private final DoubleSupplier m_hoodPositionSupplier;

    public RunFlywheelAndHood(
            Shooter shooter,
            DoubleSupplier flywheelVelocitySupplier,
            DoubleSupplier hoodPositionSupplier) {

        m_shooter = shooter;
        m_flywheelVelocitySupplier = flywheelVelocitySupplier;
        m_hoodPositionSupplier = hoodPositionSupplier;

        addRequirements(shooter);
    }

    @Override
    public void execute() {
        double flywheelVelocity = m_flywheelVelocitySupplier.getAsDouble();
        double hoodPosition = m_hoodPositionSupplier.getAsDouble();

        m_shooter.setFlywheelVelocity(flywheelVelocity);
        m_shooter.setHoodPosition(hoodPosition);
    }

    @Override
    public void end(boolean interrupted) {
        

       if (interrupted) {
        m_shooter.setHoodPercentage(0.0);
        m_shooter.setFlywheelPercentage(0.0);
    }
    }

    @Override
    public boolean isFinished() {
        return false;
    }
    
}