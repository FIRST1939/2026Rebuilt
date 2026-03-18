package frc.robot.commands.intake;

import java.util.Optional;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.intake.Intake;

public class IntakeStateManager extends Command {
    
    private final Intake m_intake;
    private State m_goalState = null;
    private Optional<State> m_overrideGoal = Optional.empty();

    private Debouncer m_debouncer;

    public enum State {
        STOWED,
        IDLE,
        EXTENDED,
        INTAKING,
        AGITATING_IN,
        AGITATING_OUT,
        DEEP_AGITATE_IN,
        REVERSING
    }

    public IntakeStateManager (Intake intake) {

        m_intake = intake;
        addRequirements(intake);
    }

    public void setGoalState (State goalState) {

        if (goalState == State.INTAKING) {

            m_debouncer = new Debouncer(0.25);
        }

        m_goalState = goalState;
    }

    public void setOverrideGoal (State goalState) {
    
        m_overrideGoal = Optional.of(goalState);
    }

    public void clearOverrideGoal () {
    
        m_overrideGoal = Optional.empty();
    }

    @Override
    public void execute () {

        State goalState;

        if (m_overrideGoal.isPresent()) { goalState = m_overrideGoal.get(); }
        else { goalState = m_goalState; }
        if (goalState == null) { return; }

        if (goalState == State.STOWED) {

            m_intake.setPivotPosition(0);
            m_intake.setRollerPercentage(0);
        } else if (goalState == State.IDLE) {

            m_intake.setPivotPosition(Constants.kPivotIdleSetpoint);
            m_intake.setRollerPercentage(0);
        } else if (goalState == State.EXTENDED) {

            m_intake.setPivotPercentage(0.1);
            m_intake.setRollerPercentage(0);
        } else if (goalState == State.INTAKING) {

            boolean down = m_debouncer.calculate(
                Math.abs(m_intake.getLeftPivotVelocity()) < 1.0 && m_intake.getLeftPivotCurrent() > 25 &&
                Math.abs(m_intake.getRightPivotVelocity()) < 1.0 && m_intake.getLeftPivotCurrent() > 25);

            if (down) { m_intake.zeroPivot(Constants.kPivotOutSetpoint); }

            m_intake.setPivotPercentage(0.25);
            m_intake.setRollerPercentage(1.0);
            //m_intake.setRollerVelocity(Constants.kBaseRollerIntakeVelocity);
        } else if (goalState == State.AGITATING_IN) {

            m_intake.setPivotPosition(Constants.kPivotLightSetpoint);
            m_intake.setRollerVelocity(Constants.kRollerAgitateVelocity);
        } else if (goalState == State.AGITATING_OUT) {

            m_intake.setPivotPosition(Constants.kPivotAgitateOutSetpoint);
            m_intake.setRollerVelocity(Constants.kRollerAgitateVelocity);
        } else if (goalState == State.DEEP_AGITATE_IN) {

            m_intake.setPivotPosition(Constants.kPivotIdleSetpoint);
            m_intake.setRollerVelocity(Constants.kRollerAgitateVelocity);
        } else if (goalState == State.REVERSING) {

            m_intake.setRollerVelocity(Constants.kRollerReverseVelocity);
        }
    }
}
