package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.subsystems.intake.Intake;

public class AgitateOnce extends SequentialCommandGroup {
    
    public AgitateOnce(Intake intake) {
        
        super(
            new RunAgitatePivot(intake, Constants.kPivotLightSetpoint, () -> Constants.kRollerAgitateVelocity),
            new RunRoller(intake, () -> Constants.kRollerAgitateVelocity).withTimeout(0.5),
            new RunAgitatePivot(intake, Constants.kPivotOutSetpoint, () -> Constants.kRollerAgitateVelocity)
        );
    }
}
