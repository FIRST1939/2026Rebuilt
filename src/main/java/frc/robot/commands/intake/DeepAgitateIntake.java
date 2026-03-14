package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.subsystems.intake.Intake;
import frc.robot.commands.intake.IntakeStateManager.State;

public class DeepAgitateIntake extends SequentialCommandGroup {
    
    public DeepAgitateIntake(Intake intake, IntakeStateManager intakeStateManager) {

        super(
            Commands.runOnce(() -> intakeStateManager.setOverrideGoal(State.DEEP_AGITATE_IN)),
            Commands.waitUntil(() -> intake.isPivotAtSetpoint(Constants.kPivotIdleSetpoint)),
            Commands.waitSeconds(0.5),
            Commands.runOnce(() -> intakeStateManager.setOverrideGoal(State.AGITATING_OUT)),
            Commands.waitUntil(() -> intake.isPivotAtSetpoint(Constants.kPivotOutSetpoint)),
            Commands.runOnce(() -> intakeStateManager.clearOverrideGoal())
        );
    }
}
