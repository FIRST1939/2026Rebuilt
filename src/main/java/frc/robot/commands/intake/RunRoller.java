package frc.robot.commands.intake;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.intake.Intake;

public class RunRoller extends Command {

    private final Intake m_intake;
    private final DoubleSupplier m_rollerVelocitySupplier;

    public RunRoller(Intake intake, DoubleSupplier rollerVelocitySupplier) {

        m_intake = intake;
        m_rollerVelocitySupplier = rollerVelocitySupplier;

        addRequirements(intake);
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        m_intake.setRollerVelocity(m_rollerVelocitySupplier.getAsDouble());
    }

    @Override
    public void end (boolean interrupted) {
       if (interrupted) {
            m_intake.setRollerPercentage(0.0);
       }
    }
}