package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.intake.Intake;
import frc.robot.commands.intake.IntakeStateManager.State;

public class ZeroAndIdleIntake extends SequentialCommandGroup {

    public ZeroAndIdleIntake(Intake intake, IntakeStateManager intakeStateManager) {

        super(
            new ZeroIntake(intake),
            Commands.runOnce(() -> intakeStateManager.setGoalState(State.IDLE))
        );
    }
}
