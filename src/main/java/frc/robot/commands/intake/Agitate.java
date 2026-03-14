package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import frc.robot.Constants;
import frc.robot.subsystems.intake.Intake;
import frc.robot.commands.intake.IntakeStateManager.State;

public class Agitate extends RepeatCommand {
    
    public Agitate(Intake intake, IntakeStateManager intakeStateManager) {

        super(
            Commands.sequence(
                Commands.runOnce(() -> intakeStateManager.setGoalState(State.AGITATING_IN)),
                Commands.waitUntil(() -> intake.isPivotAtSetpoint(Constants.kPivotLightSetpoint)),
                Commands.waitSeconds(0.5),
                Commands.runOnce(() -> intakeStateManager.setGoalState(State.AGITATING_OUT)),
                Commands.waitUntil(() -> intake.isPivotAtSetpoint(Constants.kPivotOutSetpoint)),
                Commands.waitSeconds(0.5)
            )
        );
    }
}
