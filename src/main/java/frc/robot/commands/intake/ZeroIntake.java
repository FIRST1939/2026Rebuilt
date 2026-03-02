package frc.robot.commands.intake;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.subsystems.intake.Intake;

public class ZeroIntake extends SequentialCommandGroup {
    
    private static Debouncer m_debouncer;

    public ZeroIntake(Intake intake) {

        super(
            new InstantCommand(() -> m_debouncer = new Debouncer(0.25)),
            new RunPivotPercentage(intake, Constants.kPivotZeroPercentage).until(() -> m_debouncer.calculate(
                intake.getLeftPivotVelocity() < 1.0 && intake.getLeftPivotCurrent() > 25 &&
                intake.getRightPivotVelocity() < 1.0 && intake.getLeftPivotCurrent() > 25)),
            new InstantCommand(() -> intake.zeroPivot())
        );
    }
}
