package frc.robot.commands.intake;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.intake.Intake;

public class PivotIntakePercentage extends Command {
    
    private final Intake m_intake;
    private final double m_percentage;

    public PivotIntakePercentage (Intake intake, double percentage) {

        m_intake = intake;
        m_percentage = percentage;

        addRequirements(intake);
    }

    @Override
    public void initialize () {

        m_intake.setPivotPercentage(m_percentage);
    }

    @Override
    public void end(boolean interrupted) {
        

       if (interrupted) {
            m_intake.setPivotPercentage(0.0);
        }
    }
}
