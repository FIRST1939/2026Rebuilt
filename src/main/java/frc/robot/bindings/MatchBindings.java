package frc.robot.bindings;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.*;
import frc.robot.commands.climber.*;
import frc.robot.commands.drive.*;
import frc.robot.commands.feeder.*;
import frc.robot.commands.intake.IntakeStateManager.State;
import frc.robot.commands.intake.*;
import frc.robot.commands.shooter.*;
import frc.robot.commands.spindexer.*;
import frc.robot.util.*;

public class MatchBindings {
    
    private final Debouncer m_hubAlignedDebouncer = new Debouncer(0.5, DebounceType.kFalling);

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
                () -> -bindingParams.driverController.getLeftY(),
                () -> -bindingParams.driverController.getLeftX(),
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

        modeTrigger.and(bindingParams.driverController.leftTrigger()).onTrue(
            new JoystickDriveAtAngle(
                bindingParams.drive,
                () -> -bindingParams.driverController.getLeftY(),
                () -> -bindingParams.driverController.getLeftX(),
                () -> -bindingParams.driverController.getRightX(),
                () -> bindingParams.ferrySolver.getFerrySolution().aimHeading
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
        modeTrigger.and(bindingParams.driverController.povUp()).toggleOnTrue(new RaiseClimberToHeight(bindingParams.climber, ClimberConstants.kRaisingClimberSetpoint, ClimberConstants.kRaisingClimberPercentage));
        modeTrigger.and(bindingParams.driverController.povDown()).toggleOnTrue(new LowerClimberToHeight(bindingParams.climber, ClimberConstants.kLoweringClimberSetpoint, ClimberConstants.kLoweringClimberPercentage));

        modeTrigger.and(bindingParams.operatorController.povRight()).whileTrue(
            new RunFlywheelAndHood(bindingParams.shooter, 
            () -> ShooterConstants.kOutpostFlywheelVelocity,
            () -> ShooterConstants.kOutpostHoodSetpoint));
        //Static Shot Outpost Command

        modeTrigger.and(bindingParams.operatorController.povUp()).toggleOnTrue(new RaiseClimberToHeight(bindingParams.climber, ClimberConstants.kRaisingClimberSetpoint, ClimberConstants.kRaisingClimberPercentage));
        modeTrigger.and(bindingParams.operatorController.povDown()).toggleOnTrue(new LowerClimberToHeight(bindingParams.climber, ClimberConstants.kLoweringClimberSetpoint, ClimberConstants.kLoweringClimberPercentage));

        modeTrigger.and(bindingParams.operatorController.povLeft()).whileTrue(
            new RunFlywheelAndHood(bindingParams.shooter, 
            () -> ShooterConstants.kTrenchFlywheelVelocity,
            () -> ShooterConstants.kTrenchHoodSetpoint));
        //Static Shot Trench Command

        modeTrigger.and(bindingParams.operatorController.rightTrigger()).whileTrue(
            Commands.parallel(
                new RunFlywheelAndHood(bindingParams.shooter,
                    () -> bindingParams.shotSolver.getShotSolution().flywheelRPM,
                    () -> bindingParams.shotSolver.getShotSolution().hoodPositionRotations
                ),
                Commands.sequence(
                    Commands.waitUntil(() -> bindingParams.shooter.isAtGoal()),
                    new RepeatCommand(
                        Commands.parallel(
                            new RunSpindexerPercentage(bindingParams.spindexer, SpindexerConstants.kSpindexerPercentage),
                            new RunFeederVelocity(bindingParams.feeder, FeederConstants.kFeederVelocity),
                            new AgitateIntake(bindingParams.intake, bindingParams.intakeStateManager)
                        ).onlyWhile(() -> ShiftUtil.fuelWillScore(bindingParams.shotSolver.getShotSolution().timeOfFlight) &&
                        m_hubAlignedDebouncer.calculate(bindingParams.drive.atTargetRotation(bindingParams.shotSolver.getShotSolution().aimHeading)))
                    ) // TODO Additional Shot Conditions
                )
            )
        );

        modeTrigger.and(bindingParams.operatorController.leftTrigger()).whileTrue(
            Commands.parallel(
                new RunFlywheelAndHood(bindingParams.shooter,
                    () -> bindingParams.ferrySolver.getFerrySolution().flywheelRPM,
                    () -> bindingParams.ferrySolver.getFerrySolution().hoodPositionRotations
                ),
                Commands.sequence(
                    Commands.waitUntil(() -> bindingParams.shooter.isAtGoal()),
                    new RepeatCommand(
                        Commands.parallel(
                            new RunSpindexerPercentage(bindingParams.spindexer, SpindexerConstants.kSpindexerPercentage),
                            new RunFeederVelocity(bindingParams.feeder, FeederConstants.kFeederVelocity),
                            new AgitateIntake(bindingParams.intake, bindingParams.intakeStateManager)
                        )
                    )
                )
            )
        );

        modeTrigger.and(bindingParams.operatorController.leftBumper()).onTrue(Commands.runOnce(() -> bindingParams.intakeStateManager.setGoalState(State.IDLE)));
        //Pivot Intake Idle

        modeTrigger.and(bindingParams.operatorController.start()).onTrue(Commands.runOnce(() -> bindingParams.intakeStateManager.setGoalState(State.STOWING)));
        //Pivot Intake Stow

        modeTrigger.and(bindingParams.operatorController.a()).onTrue(new DeepAgitateIntake(bindingParams.intake, bindingParams.intakeStateManager));
        //Pivot Agitate
        
        modeTrigger.and(bindingParams.operatorController.rightBumper()).onTrue(Commands.runOnce(() -> bindingParams.intakeStateManager.setMegaOverrideGoal(State.INTAKING)));
        modeTrigger.and(bindingParams.operatorController.rightBumper()).onFalse(Commands.runOnce(() -> {
            bindingParams.intakeStateManager.setGoalState(State.EXTENDED);
            bindingParams.intakeStateManager.clearMegaOverrideGoal();
        }));
        //Deploy+Roller

        modeTrigger.and(bindingParams.operatorController.b()).whileTrue(new RunSpindexerPercentage(bindingParams.spindexer, SpindexerConstants.kSpindexerReversePercentage));
        //Spindexer Reverse

        modeTrigger.and(bindingParams.operatorController.y()).whileTrue(new RunFeederVelocity(bindingParams.feeder, FeederConstants.kFeederReverseVelocity));
        //Feeder Reverse

        modeTrigger.and(bindingParams.operatorController.x()).onTrue(Commands.runOnce(() -> bindingParams.intakeStateManager.setOverrideGoal(State.REVERSING)));
        modeTrigger.and(bindingParams.operatorController.x()).onFalse(Commands.runOnce(() -> bindingParams.intakeStateManager.clearOverrideGoal()));
        //Roller Reverse
    }
}
