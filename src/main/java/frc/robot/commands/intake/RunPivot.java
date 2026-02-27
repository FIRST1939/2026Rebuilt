package frc.robot.commands.intake;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.intake.Intake;

public class RunPivot extends Command {

    private final Intake m_intake;
    private final double m_pivotPosition;

    public RunPivot(
            Intake intake,
            double pivotPosition
            ) {

        m_intake = intake;
        m_pivotPosition = pivotPosition;
       

        addRequirements(intake);
    }

    @Override
    public void initialize() {
        m_intake.setPivotPosition(m_pivotPosition);
    }

    @Override
    public void execute() {
    }

    @Override
    public boolean isFinished() {
        return m_intake.isPivotAtSetpoint();
    }

    @Override
    public void end (boolean interrupted) {
      
    }
}