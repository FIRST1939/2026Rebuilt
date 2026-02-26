package frc.robot.commands.intake;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.intake.IntakePivot;

public class AgitateIntake extends Command {

    private final IntakePivot m_intakePivot;
    private final double m_offset;
    private final double m_interval;

    private final Timer m_timer = new Timer();
    private double m_centerPosition;
    private boolean m_forward = true;

    public AgitateIntake(IntakePivot intakePivot, double offset, double intervalSeconds) {
        m_intakePivot = intakePivot;
        m_offset = offset;
        m_interval = intervalSeconds;
        addRequirements(intakePivot);
    }

    @Override
    public void initialize() {
        m_centerPosition = m_intakePivot.getPivotPosition();
        m_timer.restart();
        m_forward = true;
    }

    @Override
    public void execute() {

        if (m_timer.hasElapsed(m_interval)) {
            m_timer.restart();

            if (m_forward) {
                m_forward = false;
            } else {
                m_forward = true;
            }
        }

        double target = 0.0;

        if (m_forward) {
            target = m_centerPosition + m_offset;
        } else {
            target = m_centerPosition - m_offset;
        }

        m_intakePivot.setPivotPosition(target);
    }

    @Override
    public void end(boolean interrupted) {
        m_intakePivot.setPivotPosition(m_centerPosition);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}