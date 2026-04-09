package frc.robot.commands.shooter;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.shooter.Shooter;

public class ZeroHood extends SequentialCommandGroup {
 
    private static Debouncer m_zeroDebouncer = new Debouncer(0.25);

    public ZeroHood(Shooter shooter) {

        super(
            Commands.runOnce(() -> m_zeroDebouncer = new Debouncer(0.25)),
            new RunHoodPercentage(shooter, ShooterConstants.kHoodZeroPercentage).until(() -> 
                m_zeroDebouncer.calculate(
                    shooter.getHoodCurrent() > 3.0 && 
                    Math.abs(shooter.getHoodVelocity()) < 0.01
                )
            ),
            Commands.runOnce(() -> shooter.zeroHood(), shooter)
        );
    }
}
