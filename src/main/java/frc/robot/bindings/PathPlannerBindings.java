package frc.robot.bindings;

import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.events.EventTrigger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.*;
import frc.robot.commands.climber.LowerClimberToHeight;
import frc.robot.commands.climber.RaiseClimberToHeight;
import frc.robot.commands.drive.PPShootOnTheMoveRotation;
import frc.robot.commands.feeder.RunFeederVelocity;
import frc.robot.commands.intake.AgitateIntake;
import frc.robot.commands.intake.IntakeStateManager.State;
import frc.robot.commands.shooter.RunFlywheelAndHood;
import frc.robot.commands.spindexer.RunSpindexerPercentage;
import frc.robot.util.MapUtil.ShotSolution;

public class PathPlannerBindings {
    
    public PathPlannerBindings(BindingParams bindingParams) {

        new EventTrigger("Start Intake").onTrue(
            Commands.runOnce(() -> 
                bindingParams.intakeStateManager.setGoalState(State.INTAKING)
            )
        );

        new EventTrigger("Stop Intake").onTrue(
            Commands.runOnce(() -> 
                bindingParams.intakeStateManager.setGoalState(State.EXTENDED)
            )
        );

        new EventTrigger("Idle Intake").onTrue(
            Commands.runOnce(() -> 
                bindingParams.intakeStateManager.setGoalState(State.IDLE)
            )
        );

        new EventTrigger("Stow Intake").onTrue(
            Commands.runOnce(() -> 
                bindingParams.intakeStateManager.setGoalState(State.STOWING)
            )
        );

        ShotSolution DTrenchShotSolution = bindingParams.shotSolver.getPPShotSolution(
            new Pose2d(
                2.875, 
                7.25, 
                Rotation2d.fromDegrees(120.0)
            )
        );

        Command prepareDTrenchShotCommand = new RunFlywheelAndHood(
            bindingParams.shooter, 
            () -> DTrenchShotSolution.flywheelRPM, 
            () -> DTrenchShotSolution.hoodPositionRotations
        );

        new EventTrigger("Prepare DTrench Shot").onTrue(prepareDTrenchShotCommand);

        NamedCommands.registerCommand(
            "Stop Shot",
            Commands.runOnce(() -> {

                prepareDTrenchShotCommand.cancel();
            })
        );

        NamedCommands.registerCommand(
            "IdleIntake",
            Commands.runOnce(() -> bindingParams.intakeStateManager.setGoalState(State.IDLE))
        );

        NamedCommands.registerCommand(
            "Shoot",
            Commands.sequence(
                Commands.waitUntil(() -> bindingParams.shooter.isAtGoal()),
                Commands.parallel(
                    new AgitateIntake(bindingParams.intake, bindingParams.intakeStateManager),
                    new RunSpindexerPercentage(bindingParams.spindexer, SpindexerConstants.kSpindexerPercentage),
                    new RunFeederVelocity(bindingParams.feeder, FeederConstants.kFeederVelocity)
                )
            )
        );
        
        /*
        NamedCommands.registerCommand("RegressionShot", 
            new RunFlywheelAndHood(
                bindingParams.shooter,
                () -> bindingParams.shotSolver.getShotSolution().flywheelRPM,
                () -> bindingParams.shotSolver.getShotSolution().hoodPositionRotations
            )
        );
        */

        new EventTrigger("ShotHeading").whileTrue(
            new PPShootOnTheMoveRotation(
                bindingParams.drive,
                bindingParams.shotSolver
            )
        );

        NamedCommands.registerCommand("ClimberUp",
            new RaiseClimberToHeight(
            bindingParams.climber, 
            ClimberConstants.kRaisingClimberSetpoint, 
            ClimberConstants.kRaisingClimberPercentage)
        );

        NamedCommands.registerCommand("ClimberDown",
            new LowerClimberToHeight(
            bindingParams.climber, 
            ClimberConstants.kLoweringClimberSetpoint, 
            ClimberConstants.kLoweringClimberPercentage)
        );
    }
}
