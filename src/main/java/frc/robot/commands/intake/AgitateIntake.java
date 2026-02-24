package frc.robot.commands.intake;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.intake.Intake;

public class AgitateIntake extends Command {

    private final Intake m_intake;
    private final double m_offset;
    private final double m_interval;

    private final Timer m_timer = new Timer();
    private double m_centerPosition;
    private boolean m_forward = true;

    public AgitateIntake(Intake intake, double offset, double intervalSeconds) {
        m_intake = intake;
        m_offset = offset;
        m_interval = intervalSeconds;
        addRequirements(intake);
    }

    @Override
    public void initialize() {
        m_centerPosition = m_intake.getPivotPosition();
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

        m_intake.setPivotPosition(target);
    }

    @Override
    public void end(boolean interrupted) {
        m_intake.setPivotPosition(m_centerPosition);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}