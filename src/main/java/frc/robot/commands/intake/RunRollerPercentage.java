package frc.robot.commands.intake;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.intake.Intake;

public class RunRollerPercentage extends Command {
    
    private final Intake m_intake;
    private final double m_percentage;

    public RunRollerPercentage (Intake intake, double percentage) {

        m_intake = intake;
        m_percentage = percentage;

        addRequirements(intake);
    }

    @Override
    public void initialize () {

        m_intake.setRollerPercentage(m_percentage);
    }

    @Override
    public void end (boolean interrupted) {
        if (interrupted) {
            m_intake.setRollerPercentage(0.0);
    }
       
    }
}
