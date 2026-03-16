package frc.robot.bindings.characterization;

import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.bindings.BindingParams;

public class FeederCharacterizationBindings {
    
    public FeederCharacterizationBindings(BindingParams bindingParams, Trigger modeTrigger) {

        modeTrigger.and(bindingParams.operatorController.leftBumper()).whileTrue(bindingParams.feeder.sysIdQuasistaticForward());
        modeTrigger.and(bindingParams.operatorController.rightBumper()).whileTrue(bindingParams.feeder.sysIdQuasistaticReverse());
        modeTrigger.and(bindingParams.operatorController.leftTrigger()).whileTrue(bindingParams.feeder.sysIdDynamicForward());
        modeTrigger.and(bindingParams.operatorController.rightTrigger()).whileTrue(bindingParams.feeder.sysIdDynamicReverse());
    }
}
