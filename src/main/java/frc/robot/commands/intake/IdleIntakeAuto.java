package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.intake.Intake;

public class IdleIntakeAuto extends Command {
    
    private final Intake m_intake;

    public IdleIntakeAuto(Intake intake) {

        m_intake = intake;
        addRequirements(m_intake);
    }

    @Override
    public void initialize() {

        m_intake.setPivotPosition(Constants.kPivotInSetpoint);
    }

    @Override
    public void execute() {

        m_intake.setRollerPercentage(0);
    }

    @Override
    public boolean isFinished() {

        return m_intake.isPivotAtSetpoint(Constants.kPivotInSetpoint);
    }

    @Override
    public void end (boolean interrupted) {

        if (interrupted) {

            m_intake.setPivotPercentage(0);
        }
    }
}
