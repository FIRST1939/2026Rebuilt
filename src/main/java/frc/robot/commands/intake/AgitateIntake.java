package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import frc.robot.subsystems.intake.Intake;
import frc.robot.Constants.*;
import frc.robot.commands.intake.IntakeStateManager.State;

public class AgitateIntake extends RepeatCommand {
    
    private final IntakeStateManager m_intakeStateManager;

    public AgitateIntake(Intake intake, IntakeStateManager intakeStateManager) {

        super(
            Commands.sequence(
                Commands.runOnce(() -> intakeStateManager.setGoalState(State.AGITATING_IN)),
                Commands.waitUntil(() -> intake.isPivotAtSetpoint(IntakeConstants.kPivotAgitateInSetpoint)),
                Commands.runOnce(() -> intakeStateManager.setGoalState(State.AGITATING_OUT)),
                Commands.waitUntil(() -> intake.isPivotAtSetpoint(IntakeConstants.kPivotAgitateOutSetpoint))
            )
        );

        m_intakeStateManager = intakeStateManager;
    }

    @Override
    public void end (boolean interrupted) {

        m_intakeStateManager.setGoalState(State.EXTENDED);
    }
}
