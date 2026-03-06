package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import frc.robot.Constants;
import frc.robot.subsystems.intake.Intake;

public class Agitate extends RepeatCommand {
    
    public Agitate(Intake intake, double rollerVelocity, double pumpInterval) {

        super(
            Commands.sequence(
                new RunAgitatePivot(intake, Constants.kPivotLightSetpoint, () -> Constants.kRollerAgitateVelocity),
                new RunRoller(intake, () -> Constants.kRollerAgitateVelocity).withTimeout(0.5),
                new RunAgitatePivot(intake, Constants.kPivotOutSetpoint, () -> Constants.kRollerAgitateVelocity),
                new RunRoller(intake, () -> Constants.kRollerAgitateVelocity).withTimeout(pumpInterval)
            )
        );
    }
}
