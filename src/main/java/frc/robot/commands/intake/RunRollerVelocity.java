package frc.robot.commands.intake;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.intake.Intake;

public class RunRollerVelocity extends Command {

    private final Intake m_intake;
    private final double m_rollerVelocity;

    public RunRollerVelocity(Intake intake, double rollerVelocity) {

        m_intake = intake;
        m_rollerVelocity = rollerVelocity;

        addRequirements(intake);
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        m_intake.setRollerVelocity(m_rollerVelocity);
    }

    @Override
    public void end (boolean interrupted) {
       if (interrupted) {
            m_intake.setRollerPercentage(0.0);
       }
    }
}