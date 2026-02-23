package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.intake.Intake;

public class AgitateIntake extends Command {

    private final Intake m_intake;
    private final double m_amplitude;
    private final double m_period;   
    private double m_elapsedTime = 0.0;
    private double m_idlePosition;

    public AgitateIntake(Intake intake, double amplitude, double periodSeconds) {
        m_intake = intake;
        m_amplitude = amplitude;
        m_period = periodSeconds;
        addRequirements(intake);
    }

    @Override
    public void initialize() {
        m_idlePosition = m_intake.getPivotPosition();
        m_elapsedTime = 0.0;
    }

    @Override
    public void execute() {
        m_elapsedTime += 0.02; 
        double pivotPosition = m_idlePosition + m_amplitude * Math.sin(2 * Math.PI * m_elapsedTime / m_period);
        m_intake.setPivotPosition(pivotPosition);
    }

    @Override
    public void end(boolean interrupted) {
        m_intake.setPivotPosition(m_idlePosition);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}