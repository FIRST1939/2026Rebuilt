package frc.robot.commands.intake;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.intake.IntakeRollers;

public class RunIntakeRollerPercentage extends Command {
    
    private final IntakeRollers m_intakeRollers;
    private final double m_percentage;

    public RunIntakeRollerPercentage (IntakeRollers intakeRollers, double percentage) {

        m_intakeRollers = intakeRollers;
        m_percentage = percentage;

        addRequirements(intakeRollers);
    }

    @Override
    public void initialize () {

        m_intakeRollers.setRollerPercentage(m_percentage);
    }

    @Override
    public void end (boolean interrupted) {
       if (interrupted) { 
            m_intakeRollers.setRollerPercentage(0);
        }
    }
}
