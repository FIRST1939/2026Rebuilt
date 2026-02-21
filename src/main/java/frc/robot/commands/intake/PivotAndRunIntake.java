package frc.robot.commands.intake;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.intake.Intake;

public class PivotAndRunIntake extends Command {

    private final Intake m_intake;
    private final double m_pivotPosition;
    private final DoubleSupplier m_rollerVelocitySupplier;

    public PivotAndRunIntake(
            Intake intake,
            double pivotPosition,
            DoubleSupplier rollerVelocitySupplier) {

        m_intake = intake;
        m_pivotPosition = pivotPosition;
        m_rollerVelocitySupplier = rollerVelocitySupplier;

        addRequirements(intake);
    }

    @Override
    public void initialize() {
        m_intake.setPivotPosition(m_pivotPosition);
    }

    @Override
    public void execute() {
        m_intake.setRollerVelocity(m_rollerVelocitySupplier.getAsDouble());
    }

    @Override
    public void end(boolean interrupted) {
        m_intake.setRollerVelocity(0.0);
    }
}