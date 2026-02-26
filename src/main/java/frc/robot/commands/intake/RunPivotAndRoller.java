package frc.robot.commands.intake;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.intake.IntakePivot;
import frc.robot.subsystems.intake.IntakeRollers;

public class RunPivotAndRoller extends Command {

    private final IntakePivot m_intakePivot;
    private final IntakeRollers m_intakeRollers;
    private final double m_pivotPosition;
    private final DoubleSupplier m_rollerVelocitySupplier;

    public RunPivotAndRoller(IntakePivot intakePivot, IntakeRollers intakeRollers, double pivotPosition, DoubleSupplier rollerVelocitySupplier) {

        m_intakePivot = intakePivot;
        m_intakeRollers = intakeRollers;
        m_pivotPosition = pivotPosition;
        m_rollerVelocitySupplier = rollerVelocitySupplier;

        addRequirements(intakePivot, intakeRollers);
    }

    @Override
    public void initialize() {
        m_intakePivot.setPivotPosition(m_pivotPosition);
    }

    @Override
    public void execute() {
        m_intakeRollers.setRollerVelocity(m_rollerVelocitySupplier.getAsDouble());
    }

    @Override
    public void end (boolean interrupted) {
       if (interrupted) {
            m_intakeRollers.setRollerPercentage(0.0);
            m_intakePivot.setPivotPercentage(0.0);
       }
    }
}