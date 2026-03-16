package frc.robot.bindings.characterization;

import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.bindings.BindingParams;

public class SpindexerCharacterizationBindings {
    
    public SpindexerCharacterizationBindings(BindingParams bindingParams, Trigger modeTrigger) {

        modeTrigger.and(bindingParams.operatorController.leftBumper()).whileTrue(bindingParams.spindexer.sysIdQuasistaticForward());
        modeTrigger.and(bindingParams.operatorController.rightBumper()).whileTrue(bindingParams.spindexer.sysIdQuasistaticReverse());
        modeTrigger.and(bindingParams.operatorController.leftTrigger()).whileTrue(bindingParams.spindexer.sysIdDynamicForward());
        modeTrigger.and(bindingParams.operatorController.rightTrigger()).whileTrue(bindingParams.spindexer.sysIdDynamicReverse());
    }
}
