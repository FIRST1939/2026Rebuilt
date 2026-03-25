package frc.robot.bindings.characterization;

import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.bindings.BindingParams;
import frc.robot.commands.feeder.RunFeederPercentage;
import frc.robot.commands.spindexer.RunSpindexerPercentage;
import frc.robot.subsystems.shooter.ShooterConstants;

public class ShooterCharacterizationBindings {
    
    private final LoggedNetworkNumber m_flywheelSetpoint = new LoggedNetworkNumber("/Tuning/Shooter/Flywheel Setpoint", 0);
    private final LoggedNetworkNumber m_hoodSetpoint = new LoggedNetworkNumber("/Tuning/Shooter/Hood Setpoint", 0);

    private final LoggedNetworkNumber m_flywheelP = new LoggedNetworkNumber("/Tuning/Shooter/Flywheel P", ShooterConstants.kFlywheelFeedbackP);
    private final LoggedNetworkNumber m_flywheelD = new LoggedNetworkNumber("/Tuning/Shooter/Flywheel D", 0);

    private final LoggedNetworkNumber m_hoodP = new LoggedNetworkNumber("/Tuning/Shooter/Hood P", ShooterConstants.kHoodFeedbackP);
    private final LoggedNetworkNumber m_hoodD = new LoggedNetworkNumber("/Tuning/Shooter/Hood D", ShooterConstants.kHoodFeedbackD);

    private DoubleSupplier m_flywheelError;
    private DoubleSupplier m_hoodError;

    public ShooterCharacterizationBindings(BindingParams bindingParams, Trigger modeTrigger) {

        modeTrigger.and(bindingParams.operatorController.leftBumper()).whileTrue(bindingParams.shooter.flywheelSysIdQuasistaticForward());
        modeTrigger.and(bindingParams.operatorController.rightBumper()).whileTrue(bindingParams.shooter.flywheelSysIdQuasistaticReverse());
        modeTrigger.and(bindingParams.operatorController.leftTrigger()).whileTrue(bindingParams.shooter.flywheelSysIdDynamicForward());
        modeTrigger.and(bindingParams.operatorController.rightTrigger()).whileTrue(bindingParams.shooter.flywheelSysIdDynamicReverse());

        modeTrigger.and(bindingParams.operatorController.y()).whileTrue(bindingParams.shooter.hoodSysIdQuasistaticForward());
        modeTrigger.and(bindingParams.operatorController.b()).whileTrue(bindingParams.shooter.hoodSysIdQuasistaticReverse());
        modeTrigger.and(bindingParams.operatorController.a()).whileTrue(bindingParams.shooter.hoodSysIdDynamicForward());
        modeTrigger.and(bindingParams.operatorController.x()).whileTrue(bindingParams.shooter.hoodSysIdDynamicReverse());

        modeTrigger.and(bindingParams.operatorController.povLeft()).whileTrue(Commands.parallel(
            new RunSpindexerPercentage(bindingParams.spindexer, 0.8),
            new RunFeederPercentage(bindingParams.feeder, 0.8)
        ));

        modeTrigger.and(bindingParams.operatorController.povLeft()).onTrue(Commands.runOnce(() -> {

            bindingParams.shooter.updateFlywheelControllerFeedback(
                m_flywheelP.getAsDouble(),
                m_flywheelD.getAsDouble()
            );

            double setpoint = m_flywheelSetpoint.getAsDouble();
            m_flywheelError = () -> setpoint - bindingParams.shooter.getFlywheelVelocity();
            bindingParams.shooter.setFlywheelVelocity(setpoint);
        }, bindingParams.shooter));

        modeTrigger.and(bindingParams.operatorController.povLeft()).onFalse(Commands.runOnce(() -> {

            bindingParams.shooter.setFlywheelPercentage(0);
        }, bindingParams.shooter));

        modeTrigger.and(bindingParams.operatorController.povRight()).onTrue(Commands.runOnce(() -> {

            bindingParams.shooter.updateHoodControllerFeedback(
                m_hoodP.getAsDouble(),
                m_hoodD.getAsDouble()
            );

            double setpoint = m_hoodSetpoint.getAsDouble();
            m_hoodError = () -> setpoint - bindingParams.shooter.getHoodPosition();
            bindingParams.shooter.setHoodPosition(setpoint);
        }, bindingParams.shooter));

        modeTrigger.and(bindingParams.operatorController.povRight()).onFalse(Commands.runOnce(() -> {

            bindingParams.shooter.setHoodPercentage(0);
        }, bindingParams.shooter));
    }

    @AutoLogOutput(key = "/Tuning/Shooter/Flywheel Error")
    private double getFlywheelError() {

        return m_flywheelError.getAsDouble();
    }

    @AutoLogOutput(key = "/Tuning/Shooter/Hood Error")
    private double getHoodError() {

        return m_hoodError.getAsDouble();
    }
}
