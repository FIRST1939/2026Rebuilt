package frc.robot.commands.intake;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.intake.IntakeRollers;

public class RunRoller extends Command {

    private final IntakeRollers m_intakeRollers;
    private final DoubleSupplier m_rollerVelocitySupplier;

    public RunRoller(IntakeRollers intakeRollers, DoubleSupplier rollerVelocitySupplier) {

        m_intakeRollers = intakeRollers;
        m_rollerVelocitySupplier = rollerVelocitySupplier;

        addRequirements(intakeRollers);
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        m_intakeRollers.setRollerVelocity(m_rollerVelocitySupplier.getAsDouble());
    }

    @Override
    public void end (boolean interrupted) {
       if (interrupted) {
            m_intakeRollers.setRollerPercentage(0.0);
       }
    }
}