package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.subsystems.intake.Intake;

public class ZeroAndIdleIntake extends SequentialCommandGroup {

    public ZeroAndIdleIntake(Intake intake) {

        super(
            new ZeroIntake(intake),
            new RunPivot(intake, Constants.kPivotInSetpoint)
        );
    }
}
