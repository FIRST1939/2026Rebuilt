package frc.robot.commands.intake;

import java.util.Optional;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.IntakeConstants;
import frc.robot.subsystems.intake.Intake;

public class IntakeStateManager extends Command {
    
    private final Intake m_intake;
    private State m_goalState = State.STOWED;
    private Optional<State> m_overrideGoal = Optional.empty();
    private Optional<State> m_megaOverrideGoal = Optional.empty();

    private Debouncer m_debouncer;

    public enum State {
        STOWED,
        STOWING,
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

    public State getGoalState () {

        return m_goalState;
    }

    public void setGoalState (State goalState) {

        if (goalState == State.INTAKING || goalState == State.STOWING) {

            m_debouncer = new Debouncer(0.25);
        }

        m_goalState = goalState;
    }

    public void setOverrideGoal (State goalState) {
    
        if (goalState == State.INTAKING || goalState == State.STOWING) {

            m_debouncer = new Debouncer(0.25);
        }

        m_overrideGoal = Optional.of(goalState);
    }

    public void clearOverrideGoal () {
    
        m_overrideGoal = Optional.empty();
    }

    public void setMegaOverrideGoal (State goalState) {

        if (goalState == State.INTAKING || goalState == State.STOWING) {

            m_debouncer = new Debouncer(0.25);
        }

        m_megaOverrideGoal = Optional.of(goalState);
    }

    public void clearMegaOverrideGoal () {

        m_megaOverrideGoal = Optional.empty();
    }

    @Override
    public void execute () {

        State goalState = m_goalState;

        if (m_overrideGoal.isPresent()) { goalState = m_overrideGoal.get(); }
        if (m_megaOverrideGoal.isPresent()) { goalState = m_megaOverrideGoal.get(); }
        if (goalState == null) { return; }

        Logger.recordOutput("Intake State", goalState.toString());

        if (goalState == State.STOWED) {

            m_intake.setPivotPercentage(0);
            m_intake.setRollerPercentage(0);
        } else if (goalState == State.STOWING) {

            boolean stowed = m_debouncer.calculate(
                Math.abs(m_intake.getLeftPivotVelocity()) < 1.0 && m_intake.getLeftPivotCurrent() > 25 &&
                Math.abs(m_intake.getRightPivotVelocity()) < 1.0 && m_intake.getLeftPivotCurrent() > 25
            );

            if (stowed) { setGoalState(State.STOWED); }

            m_intake.setPivotPercentage(IntakeConstants.kPivotStowingPercentage);
            m_intake.setRollerPercentage(0);
        } else if (goalState == State.IDLE) {

            m_intake.setPivotPosition(IntakeConstants.kPivotIdleSetpoint);
            m_intake.setRollerPercentage(0);
        } else if (goalState == State.EXTENDED) {

            m_intake.setPivotPercentage(IntakeConstants.kPivotExtendedPercentage);
            m_intake.setRollerPercentage(0);
        } else if (goalState == State.INTAKING) {

            boolean down = m_debouncer.calculate(
                Math.abs(m_intake.getLeftPivotVelocity()) < 1.0 && m_intake.getLeftPivotCurrent() > 25 &&
                Math.abs(m_intake.getRightPivotVelocity()) < 1.0 && m_intake.getLeftPivotCurrent() > 25
            );

            if (down) { m_intake.zeroPivot(IntakeConstants.kPivotOutSetpoint); }

            m_intake.setPivotPercentage(IntakeConstants.kPivotIntakePercentage);
            m_intake.setRollerPercentage(IntakeConstants.kRollerIntakePercentage);
        } else if (goalState == State.AGITATING_IN) {

            m_intake.setPivotPosition(IntakeConstants.kPivotAgitateInSetpoint);
            m_intake.setRollerVelocity(IntakeConstants.kRollerAgitateVelocity);
        } else if (goalState == State.AGITATING_OUT) {

            m_intake.setPivotPosition(IntakeConstants.kPivotAgitateOutSetpoint);
            m_intake.setRollerVelocity(IntakeConstants.kRollerAgitateVelocity);
        } else if (goalState == State.DEEP_AGITATE_IN) {

            m_intake.setPivotPosition(IntakeConstants.kPivotIdleSetpoint);
            m_intake.setRollerVelocity(IntakeConstants.kRollerAgitateVelocity);
        } else if (goalState == State.REVERSING) {

            m_intake.setRollerVelocity(IntakeConstants.kRollerReverseVelocity);
        }
    }
}
