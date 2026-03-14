package frc.robot.commands.intake;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.intake.Intake;

public class RunRollerVelocity extends Command {

    private final Intake m_intake;
    private final DoubleSupplier m_rollerVelocity;

    public RunRollerVelocity(Intake intake, DoubleSupplier rollerVelocity) {

        m_intake = intake;
        m_rollerVelocity = rollerVelocity;

        addRequirements(intake);
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        m_intake.setRollerVelocity(m_rollerVelocity.getAsDouble());
    }

    @Override
    public void end (boolean interrupted) {
       if (interrupted) {
            m_intake.setRollerPercentage(0.0);
       }
    }
}