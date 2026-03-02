package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.subsystems.intake.Intake;

public class IdleAndZeroIntake extends SequentialCommandGroup {

    public IdleAndZeroIntake(Intake intake) {

        super(
            new RunPivot(intake, Constants.kPivotInSetpoint),
            new ZeroIntake(intake),
            new RunPivot(intake, Constants.kPivotInSetpoint)
        );
    }
}
