package frc.robot.commands.intake;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.intake.IntakePivot;

public class PivotIntake extends Command {

    private final IntakePivot m_intakePivot;
    private final double m_pivotPosition;

    public PivotIntake(
            IntakePivot intakePivot,
            double pivotPosition
            ) {

        m_intakePivot = intakePivot;
        m_pivotPosition = pivotPosition;
       

        addRequirements(intakePivot);
    }

    @Override
    public void initialize() {
        m_intakePivot.setPivotPosition(m_pivotPosition);
    }

    @Override
    public void execute() {
    }

    @Override
    public boolean isFinished() {
        return m_intakePivot.isPivotAtSetpoint();
    }

    @Override
    public void end (boolean interrupted) {
      
    }
}