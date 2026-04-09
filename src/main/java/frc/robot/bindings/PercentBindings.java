package frc.robot.bindings;

import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.ClimberConstants;
import frc.robot.commands.climber.SetClimberPercentage;
import frc.robot.commands.feeder.RunFeederPercentage;
import frc.robot.commands.intake.RunPivotPercentage;
import frc.robot.commands.intake.RunRollerPercentage;
import frc.robot.commands.shooter.RunFlywheelPercentage;
import frc.robot.commands.shooter.RunHoodPercentage;
import frc.robot.commands.spindexer.RunSpindexerPercentage;

public class PercentBindings {
    
    public PercentBindings(BindingParams bindingParams, Trigger modeTrigger) {

        modeTrigger.and(bindingParams.operatorController.a()).whileTrue(new RunSpindexerPercentage(bindingParams.spindexer, 0.8));
        modeTrigger.and(bindingParams.operatorController.x()).whileTrue(new RunFeederPercentage(bindingParams.feeder, 0.8));
        modeTrigger.and(bindingParams.operatorController.y()).whileTrue(new RunFlywheelPercentage(bindingParams.shooter, 0.3));
        modeTrigger.and(bindingParams.operatorController.b()).whileTrue(new RunHoodPercentage(bindingParams.shooter, -0.2));
        modeTrigger.and(bindingParams.operatorController.leftTrigger()).whileTrue(new RunFlywheelPercentage(bindingParams.shooter, 0.55));
        modeTrigger.and(bindingParams.operatorController.rightTrigger()).whileTrue(new RunRollerPercentage(bindingParams.intake, 0.225));
        modeTrigger.and(bindingParams.operatorController.leftBumper()).whileTrue(new RunPivotPercentage(bindingParams.intake, -0.25));
        modeTrigger.and(bindingParams.operatorController.rightBumper()).whileTrue(new RunPivotPercentage(bindingParams.intake, 0.25));
        modeTrigger.and(bindingParams.driverController.povUp()).whileTrue(new SetClimberPercentage(bindingParams.climber, ClimberConstants.kRaisingClimberPercentage));
        modeTrigger.and(bindingParams.driverController.povDown()).whileTrue(new SetClimberPercentage(bindingParams.climber, ClimberConstants.kLoweringClimberPercentage));
    }
}
