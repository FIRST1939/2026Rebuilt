package frc.robot.bindings.config;

import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.*;
import frc.robot.bindings.BindingParams;
import frc.robot.commands.feeder.RunFeederVelocity;
import frc.robot.commands.intake.AgitateIntake;
import frc.robot.commands.shooter.RunFlywheelAndHood;
import frc.robot.commands.spindexer.RunSpindexerPercentage;

public class ShotConfigBindings {
    
    private final LoggedNetworkNumber m_flywheelRPM = new LoggedNetworkNumber("/Tuning/Shot Config/Flywheel RPM", 3000);
    private final LoggedNetworkNumber m_hoodAngleDegrees = new LoggedNetworkNumber("/Tuning/Shot Config/Hood Angle Degrees", 0);

    public ShotConfigBindings(BindingParams bindingParams, Trigger modeTrigger) {

        modeTrigger.and(bindingParams.operatorController.x()).onTrue(Commands.runOnce(() -> {

            m_flywheelRPM.set(m_flywheelRPM.get() - 50.0);
        }, bindingParams.shooter));

        modeTrigger.and(bindingParams.operatorController.b()).onTrue(Commands.runOnce(() -> {

            m_flywheelRPM.set(m_flywheelRPM.get() + 50.0);
        }, bindingParams.shooter));

        modeTrigger.and(bindingParams.operatorController.a()).onTrue(Commands.runOnce(() -> {

            m_hoodAngleDegrees.set(m_hoodAngleDegrees.get() - 0.5);
        }, bindingParams.shooter));

        modeTrigger.and(bindingParams.operatorController.y()).onTrue(Commands.runOnce(() -> {

            m_hoodAngleDegrees.set(m_hoodAngleDegrees.get() + 0.5);
        }, bindingParams.shooter));

        modeTrigger.and(bindingParams.operatorController.leftTrigger()).whileTrue(
            new RunFlywheelAndHood(bindingParams.shooter,
                () -> m_flywheelRPM.get(),
                () -> m_hoodAngleDegrees.get() / 360.0
            )
        );

        modeTrigger.and(bindingParams.operatorController.rightTrigger()).whileTrue((
            Commands.parallel(
                new RunSpindexerPercentage(bindingParams.spindexer, SpindexerConstants.kSpindexerPercentage),
                new RunFeederVelocity(bindingParams.feeder, FeederConstants.kFeederVelocity),
                new AgitateIntake(bindingParams.intake, bindingParams.intakeStateManager)
            )
        ));
    }
}
