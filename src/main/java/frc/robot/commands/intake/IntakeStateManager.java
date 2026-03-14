package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.intake.Intake;

public class IntakeStateManager extends Command {
    
    private final Intake m_intake;
    private State m_goalState = State.STOWED;

    public enum State {
        STOWED,
        IDLE,
        EXTENDED,
        INTAKING
    }

    public IntakeStateManager (Intake intake) {

        m_intake = intake;
        addRequirements(intake);
    }

    public void setGoalState (State goalState) {

        m_goalState = goalState;
    }

    @Override
    public void execute () {

        if (m_goalState == State.STOWED) {

            m_intake.setPivotPosition(0);
            m_intake.setRollerPercentage(0);
        } else if (m_goalState == State.IDLE) {

            m_intake.setPivotPosition(Constants.kPivotIdleSetpoint);
            m_intake.setRollerPercentage(0);
        } else if (m_goalState == State.EXTENDED) {

            m_intake.setPivotPosition(Constants.kPivotOutSetpoint);
            m_intake.setRollerPercentage(0);
        } else if (m_goalState == State.INTAKING) {

            m_intake.setPivotPosition(Constants.kPivotOutSetpoint);
            m_intake.setRollerVelocity(Constants.kBaseRollerIntakeVelocity);
        }
    }
}
