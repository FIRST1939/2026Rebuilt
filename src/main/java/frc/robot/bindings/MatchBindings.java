package frc.robot.bindings;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;
import frc.robot.commands.climber.*;
import frc.robot.commands.drive.*;
import frc.robot.commands.feeder.*;
import frc.robot.commands.intake.IntakeStateManager.State;
import frc.robot.commands.intake.*;
import frc.robot.commands.shooter.*;
import frc.robot.commands.spindexer.*;
import frc.robot.util.*;

public class MatchBindings {
    
    public MatchBindings(BindingParams bindingParams, Trigger modeTrigger) {

        Command defaultDriveCommand = new JoystickDrive(
            bindingParams.drive,
            () -> -bindingParams.driverController.getLeftY(),
            () -> -bindingParams.driverController.getLeftX(),
            () -> -bindingParams.driverController.getRightX()
        ).onlyWhile(modeTrigger);

        Command defaultIntakeCommand = bindingParams.intakeStateManager.onlyWhile(modeTrigger);
        
        bindingParams.drive.setDefaultCommand(defaultDriveCommand); 
        bindingParams.intake.setDefaultCommand(defaultIntakeCommand);

        modeTrigger.onTrue(
            Commands.runOnce(() -> {
                bindingParams.drive.setDefaultCommand(defaultDriveCommand);
                bindingParams.intake.setDefaultCommand(defaultIntakeCommand);
            })
        );

        modeTrigger.onFalse(
            Commands.runOnce(() -> {
                bindingParams.drive.removeDefaultCommand();
                bindingParams.intake.removeDefaultCommand();
            })
        );

        modeTrigger.and(bindingParams.driverController.rightBumper()).onTrue(
            new SnakeDrive(
                bindingParams.drive, 
                () -> -bindingParams.driverController.getLeftY() * 0.6,
                () -> -bindingParams.driverController.getLeftX() * 0.6,
                () -> -bindingParams.driverController.getRightX()
            )
        );

        modeTrigger.and(bindingParams.driverController.rightTrigger()).onTrue(
            new JoystickDriveAtAngle(
                bindingParams.drive,
                () -> -bindingParams.driverController.getLeftY(),
                () -> -bindingParams.driverController.getLeftX(),
                () -> -bindingParams.driverController.getRightX(),
                () -> bindingParams.shotSolver.getShotSolution().aimHeading
            )
        );

        modeTrigger.and(bindingParams.driverController.a()).onTrue(
            new JoystickDriveAtAngle(
                bindingParams.drive,
                () -> -bindingParams.driverController.getLeftY(),
                () -> -bindingParams.driverController.getLeftX(),
                () -> -bindingParams.driverController.getRightX(),
                () -> Util.isRedAlliance() ? new Rotation2d() : new Rotation2d(Math.PI)
            )
        );

        modeTrigger.and(bindingParams.driverController.y()).onTrue(
            new JoystickDriveAtAngle(
                bindingParams.drive,
                () -> -bindingParams.driverController.getLeftY(),
                () -> -bindingParams.driverController.getLeftX(),
                () -> -bindingParams.driverController.getRightX(),
                () -> Util.isRedAlliance() ? new Rotation2d(Math.PI) : new Rotation2d()
            )
        );

        modeTrigger.and(bindingParams.driverController.b()).onTrue(
            new JoystickDriveAtAngle(
                bindingParams.drive, 
                () -> -bindingParams.driverController.getLeftY(),
                () -> -bindingParams.driverController.getLeftX(),
                () -> -bindingParams.driverController.getRightX(),
                () -> (bindingParams.drive.getPose().getY() > FieldConstants.Tower.centerPoint.getY()) ? new Rotation2d(Math.PI / 2.0) : new Rotation2d(-Math.PI / 2.0)
            )
        );

        modeTrigger.and(bindingParams.driverController.x()).onTrue(Commands.runOnce(bindingParams.drive::stopWithX, bindingParams.drive));
        modeTrigger.and(bindingParams.driverController.povUp()).toggleOnTrue(new RaiseClimberToHeight(bindingParams.climber, Constants.kRaisingClimberSetpoint, Constants.kRaisingClimberPercentage));
        modeTrigger.and(bindingParams.driverController.povDown()).toggleOnTrue(new LowerClimberToHeight(bindingParams.climber, Constants.kLoweringClimberSetpoint, Constants.kLoweringClimberPercentage));

        modeTrigger.and(bindingParams.operatorController.povRight()).whileTrue(
            new RunFlywheelAndHood(bindingParams.shooter, 
            () -> Constants.kOutpostFlywheelVelocity,
            () -> Constants.kOutpostHoodSetpoint));
        //Static Shot Outpost Command

        modeTrigger.and(bindingParams.operatorController.leftTrigger()).whileTrue(
            new RunFlywheelAndHood(bindingParams.shooter,
                () -> bindingParams.shotSolver.getShotSolution().flywheelRPM,
                () -> bindingParams.shotSolver.getShotSolution().hoodPositionRotations
            )
        );

        modeTrigger.and(bindingParams.operatorController.povUp()).whileTrue(
            new RunFlywheelAndHood(bindingParams.shooter, 
            () -> Constants.kHubFlywheelVelocity,
            () -> Constants.kHubHoodSetpoint));
        //Static Shot Hub Command

        modeTrigger.and(bindingParams.operatorController.povDown()).whileTrue(
            new RunFlywheelAndHood(bindingParams.shooter, 
            () -> Constants.kTowerFlywheelVelocity,
            () -> Constants.kTowerHoodSetpoint));
        //Static Shot Tower Command

        modeTrigger.and(bindingParams.operatorController.povLeft()).whileTrue(
            new RunFlywheelAndHood(bindingParams.shooter, 
            () -> Constants.kTrenchFlywheelVelocity,
            () -> Constants.kTrenchHoodSetpoint));
        //Static Shot Trench Command

        modeTrigger.and(bindingParams.operatorController.rightTrigger()).whileTrue((
            new RunSpindexerVelocity(bindingParams.spindexer, Constants.kSpindexerVelocity))
            .alongWith(new RunFeederVelocity(bindingParams.feeder, Constants.kFeederVelocity))
            .alongWith(new AgitateIntake(bindingParams.intake, bindingParams.intakeStateManager)));
        //Feed Into Shooter Command

        modeTrigger.and(bindingParams.operatorController.leftBumper()).onTrue(Commands.runOnce(() -> bindingParams.intakeStateManager.setGoalState(State.IDLE)));
        //Pivot Intake In

        modeTrigger.and(bindingParams.operatorController.start()).onTrue(new ZeroAndIdleIntake(bindingParams.intake, bindingParams.intakeStateManager)); 
        //Pivot Intake Stow

        //modeTrigger.and(operatorController.a()).whileTrue(new AgitateIntake(intake, Constants.kAgitateIntakeInterval, Constants.kRollerAgitateVelocity));

        //modeTrigger.and(operatorController.a()).whileTrue(new Agitate(intake, Constants.kAgitateIntakeInterval));

        modeTrigger.and(bindingParams.operatorController.a()).onTrue(new DeepAgitateIntake(bindingParams.intake, bindingParams.intakeStateManager));
        
        modeTrigger.and(bindingParams.operatorController.rightBumper()).onTrue(Commands.runOnce(() -> bindingParams.intakeStateManager.setGoalState(State.INTAKING)));
        modeTrigger.and(bindingParams.operatorController.rightBumper()).onFalse(Commands.runOnce(() -> bindingParams.intakeStateManager.setGoalState(State.EXTENDED)));
        //Deploy+Roller

        modeTrigger.and(bindingParams.operatorController.b()).whileTrue(new RunSpindexerVelocity(bindingParams.spindexer, Constants.kSpindexerReverseVelocity));
        //Spindexer Reverse

        modeTrigger.and(bindingParams.operatorController.y()).whileTrue(new RunFeederVelocity(bindingParams.feeder, Constants.kFeederReverseVelocity));
        //Feeder Reverse

        modeTrigger.and(bindingParams.operatorController.x()).whileTrue(new RunRollerVelocity(bindingParams.intake, () -> Constants.kRollerReverseVelocity));
        //Roller Reverse
    }
}
