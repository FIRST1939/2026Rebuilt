package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.subsystems.intake.Intake;

public class Agitate extends RepeatCommand {
    
    public Agitate(Intake intake, double rollerVelocity, double pumpInterval) {

        super(
            Commands.sequence(
                new RunAgitatePivot(intake, Constants.kPivotIdleSetpoint, () -> Constants.kRollerAgitateVelocity).withTimeout(0.75),
                new WaitCommand(0.5),
                new RunAgitatePivot(intake, Constants.kPivotOutSetpoint, () -> Constants.kRollerAgitateVelocity).withTimeout(0.75),
                new WaitCommand(pumpInterval)
            )
        );
    }
}
