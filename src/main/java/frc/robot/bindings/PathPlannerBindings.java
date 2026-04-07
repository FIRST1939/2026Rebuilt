package frc.robot.bindings;

import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.events.EventTrigger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import frc.robot.Constants.*;
import frc.robot.commands.climber.LowerClimberToHeight;
import frc.robot.commands.climber.RaiseClimberToHeight;
import frc.robot.commands.feeder.RunFeederVelocity;
import frc.robot.commands.intake.AgitateIntake;
import frc.robot.commands.intake.DeepAgitateIntake;
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

        ShotSolution dTrenchShotSolution = bindingParams.shotSolver.getPPShotSolution(
            new Pose2d(
                2.875, 
                7.25, 
                Rotation2d.fromDegrees(120.0)
            )
        );

        Command prepareDTrenchShotCommand = new RunFlywheelAndHood(
            bindingParams.shooter, 
            () -> dTrenchShotSolution.flywheelRPM, 
            () -> dTrenchShotSolution.hoodPositionRotations
        );

        ShotSolution dMidShotSolution = bindingParams.shotSolver.getPPShotSolution(
            new Pose2d(
                2.609, 
                6.865, 
                Rotation2d.fromDegrees(130.0)
            )
        );

        Command prepareDMidShotCommand = new RunFlywheelAndHood(
            bindingParams.shooter, 
            () -> dMidShotSolution.flywheelRPM, 
            () -> dMidShotSolution.hoodPositionRotations
        );


        ShotSolution depotShotSolution = bindingParams.shotSolver.getPPShotSolution(
            new Pose2d(
                1.702, 
                5.84, 
                Rotation2d.fromDegrees(150.0)
            )
        );

        Command prepareDepotShotCommand = new RunFlywheelAndHood(
            bindingParams.shooter, 
            () -> depotShotSolution.flywheelRPM, 
            () -> depotShotSolution.hoodPositionRotations
        );

        new EventTrigger("Prepare DTrench Shot").onTrue(prepareDTrenchShotCommand);
        new EventTrigger("Prepare DMid Shot").onTrue(prepareDMidShotCommand);
        new EventTrigger("Prepare Depot Shot").onTrue(prepareDepotShotCommand);

        NamedCommands.registerCommand(
            "Stop Shot",
            Commands.runOnce(() -> {

                prepareDTrenchShotCommand.cancel();
                prepareDMidShotCommand.cancel();
                prepareDepotShotCommand.cancel();
            })
        );

        NamedCommands.registerCommand(
            "Shoot",
            Commands.sequence(
                Commands.waitUntil(() -> bindingParams.shooter.isAtGoal()),
                Commands.parallel(
                    new RunSpindexerPercentage(bindingParams.spindexer, SpindexerConstants.kSpindexerPercentage),
                    new RunFeederVelocity(bindingParams.feeder, FeederConstants.kFeederVelocity),
                    new AgitateIntake(bindingParams.intake, bindingParams.intakeStateManager),
                    new RepeatCommand(
                        Commands.sequence(
                            Commands.waitSeconds(1.5),
                            new DeepAgitateIntake(bindingParams.intake, bindingParams.intakeStateManager)
                        )
                    )
                )
            )
        );

        NamedCommands.registerCommand("Raise Climber",
            new RaiseClimberToHeight(
            bindingParams.climber, 
            ClimberConstants.kRaisingClimberSetpoint, 
            ClimberConstants.kRaisingClimberPercentage)
        );

        NamedCommands.registerCommand("Lower Climber",
            new LowerClimberToHeight(
            bindingParams.climber, 
            ClimberConstants.kLoweringClimberSetpoint, 
            ClimberConstants.kLoweringClimberPercentage)
        );
        
        /*
        NamedCommands.registerCommand("RegressionShot", 
            new RunFlywheelAndHood(
                bindingParams.shooter,
                () -> bindingParams.shotSolver.getShotSolution().flywheelRPM,
                () -> bindingParams.shotSolver.getShotSolution().hoodPositionRotations
            )
        );

        new EventTrigger("ShotHeading").whileTrue(
            new PPShootOnTheMoveRotation(
                bindingParams.drive,
                bindingParams.shotSolver
            )
        );
        */
    }
}
