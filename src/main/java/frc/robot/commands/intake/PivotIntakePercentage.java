package frc.robot.commands.intake;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.intake.IntakePivot;

public class PivotIntakePercentage extends Command {
    
    private final IntakePivot m_intakePivot;
    private final double m_percentage;

    public PivotIntakePercentage (IntakePivot intakePivot, double percentage) {

        m_intakePivot = intakePivot;
        m_percentage = percentage;

        addRequirements(intakePivot);
    }

    @Override
    public void initialize () {

        m_intakePivot.setPivotPercentage(m_percentage);
    }

    @Override
    public void end(boolean interrupted) {
        

       if (interrupted) {
            m_intakePivot.setPivotPercentage(0.0);
        }
    }
}
