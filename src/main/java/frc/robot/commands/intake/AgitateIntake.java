package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import frc.robot.subsystems.intake.Intake;
import frc.robot.Constants.*;
import frc.robot.commands.intake.IntakeStateManager.State;

public class AgitateIntake extends Command {
    
    private final IntakeStateManager m_intakeStateManager;

    public AgitateIntake(Intake intake, IntakeStateManager intakeStateManager) {
        
        m_intakeStateManager = intakeStateManager;
    }


     @Override
    public void initialize () {

        m_intakeStateManager.setGoalState(State.AGITATING);
    }

    @Override
    public void end (boolean interrupted) {

        m_intakeStateManager.setGoalState(State.EXTENDED);
    }
}
