package frc.robot.bindings;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import org.littletonrobotics.junction.Logger;

import frc.robot.Constants;
import frc.robot.subsystems.intake.IntakePivot;
import frc.robot.subsystems.intake.IntakeRollers;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShooterSolutionFinder;
import frc.robot.subsystems.shooter.FixedShooterSolutionFinder;
import frc.robot.subsystems.shooter.IdleShooterSolutionFinder;
import frc.robot.subsystems.shooter.InterpolatingShooterSolutionFinder;
import frc.robot.subsystems.shooter.QuickShotShooterSolutionFinder;
import frc.robot.subsystems.feeder.Feeder;
import frc.robot.subsystems.spindexer.Spindexer;
import frc.robot.subsystems.climber.Climber;
import frc.robot.commands.intake.PivotIntake;
import frc.robot.commands.intake.RunRoller;
import frc.robot.commands.shooter.FollowShooterSetpoints;
import frc.robot.commands.shooter.RunFlywheelAndHood;
import frc.robot.commands.shooter.ShootSequence;
import frc.robot.commands.feeder.RunFeederVelocity;
import frc.robot.commands.spindexer.RunSpindexerVelocity;

public class ExampleMatchBindings {

    //TODO move this to Constants.
    private static final double ROLLER_VELOCITY = 1500.0;
    private static final double ROLLER_VELOCITY_INWARD = -1000.0;
    private static final double FEEDER_RPM = 2000;
    private static final double FEEDER_THRESHOLD = 500;
    private static final double SPINDEXER_RPM = 120;
    private static final double STATIC_SHOT_RPM = 3500;
    private static final double STATIC_SHOT_HOOD = 0.05;
    private static final double CLIMBER_DEADBAND = 0.1;
    private static final double kPivotIdleSetpoint = 0.8; // example setpoint for pivoting intake out

    private static enum SolutionType { FIXED, IDLE, INTERPOLATING, QUICKSHOT }
    private static SolutionType m_activeSolutionType = SolutionType.FIXED;

    private static final ShooterSolutionFinder m_fixedFinder = new FixedShooterSolutionFinder();
    private static final ShooterSolutionFinder m_idleFinder = new IdleShooterSolutionFinder();
    private static final ShooterSolutionFinder m_interpolatingFinder = new InterpolatingShooterSolutionFinder();
    private static final ShooterSolutionFinder m_quickShotFinder = new QuickShotShooterSolutionFinder();

    private static ShooterSolutionFinder getActiveSolutionFinder() {
        switch (m_activeSolutionType) {
            case IDLE: return m_idleFinder;
            case INTERPOLATING: return m_interpolatingFinder;
            case QUICKSHOT: return m_quickShotFinder;
            case FIXED:
            default: return m_fixedFinder;
        }
    }

    public static void configure(
            Trigger exampleMatchMode,
            CommandXboxController controller,
            IntakePivot intakePivot,
            IntakeRollers intakeRollers,
            Shooter shooter,
            Feeder feeder,
            Spindexer spindexer,
            Climber climber) {

        // Right trigger: pivot intake out (stays at position after release) and run intake roller (stops on release)

        //Test: Press right trigger to pivot intake out and run rollers, release to stop rollers and leave intake in position.  
        //Press button again to start back up. 
        exampleMatchMode.and(controller.rightTrigger()).toggleOnTrue(
            new PivotIntake(intakePivot, Constants.kPivotOutSetpoint)
        );
        
        exampleMatchMode.and(controller.rightTrigger()).whileTrue(
            new RunRoller(intakeRollers, () -> ROLLER_VELOCITY)
        );

        //Right bumper: pivot intake in (stays at position after release) and run intake roller inward (stops on release)

        //Test: Press right bumper to pivot intake in and run rollers, release to stop rollers and leave intake in position.  
        //Press button again to start back up. 

        exampleMatchMode.and(controller.rightBumper()).toggleOnTrue(
            new PivotIntake(intakePivot, kPivotIdleSetpoint)
        );
        
        exampleMatchMode.and(controller.rightBumper()).whileTrue(
            new RunRoller(intakeRollers, () -> ROLLER_VELOCITY_INWARD)
        );

        // Left trigger: follow shooter setpoints AND run shoot sequence while held
        //Test: Hold left trigger to spin up the shooter following the active solution, then feed and spindex once at goal. Release to stop.
        exampleMatchMode.and(controller.leftTrigger()).whileTrue(
            Commands.parallel(
                new FollowShooterSetpoints(shooter,
                    () -> getActiveSolutionFinder().getLatestSolution().flywheelRPM,
                    () -> getActiveSolutionFinder().getLatestSolution().hoodPositionRotations),
                ShootSequence.create(shooter, feeder, spindexer, FEEDER_RPM, FEEDER_THRESHOLD, SPINDEXER_RPM)
            ).finallyDo(() -> {
                shooter.setHoodPosition(0);
                shooter.setFlywheelPercentage(0);
            })
        );

        // Left bumper: spin up shooter following the active solution finder's setpoints
        //Test: Hold left bumper to spin up the shooter. Release to stop. Hood and flywheel follow the selected solution.
        exampleMatchMode.and(controller.leftBumper()).toggleOnTrue(
            new FollowShooterSetpoints(shooter,
                () -> getActiveSolutionFinder().getLatestSolution().flywheelRPM,
                () -> getActiveSolutionFinder().getLatestSolution().hoodPositionRotations)
            .finallyDo(() -> {
                shooter.setHoodPosition(0);
                shooter.setFlywheelPercentage(0);
            })
        );

        // D-pad: select the active shooter solution finder
        //Test: Press d-pad up for Fixed, d-pad right for Interpolating, d-pad down for QuickShot.
        exampleMatchMode.and(controller.povUp()).onTrue(
            Commands.runOnce(() -> {
                m_activeSolutionType = SolutionType.FIXED;
                Logger.recordOutput("ExampleMatch/SolutionType", m_activeSolutionType.name());
            })
        );

        exampleMatchMode.and(controller.povRight()).onTrue(
            Commands.runOnce(() -> {
                m_activeSolutionType = SolutionType.INTERPOLATING;
                Logger.recordOutput("ExampleMatch/SolutionType", m_activeSolutionType.name());
            })
        );

        exampleMatchMode.and(controller.povLeft()).onTrue(
            Commands.runOnce(() -> {
                m_activeSolutionType = SolutionType.QUICKSHOT;
                Logger.recordOutput("ExampleMatch/SolutionType", m_activeSolutionType.name());
            })
        );

        exampleMatchMode.and(controller.povDown()).onTrue(
            Commands.runOnce(() -> {
                m_activeSolutionType = SolutionType.IDLE;
                Logger.recordOutput("ExampleMatch/SolutionType", m_activeSolutionType.name());
            })
        );

        // Y button: static shot — spin up flywheel + hood, run feeder and spindexer, all while held
        //Test: Hold Y to fire a static shot at fixed RPM/hood. Release to stop everything.
        exampleMatchMode.and(controller.y()).whileTrue(
            Commands.parallel(
                new RunFlywheelAndHood(shooter, () -> STATIC_SHOT_RPM, () -> STATIC_SHOT_HOOD),
                new RunFeederVelocity(feeder, FEEDER_RPM),
                new RunSpindexerVelocity(spindexer, SPINDEXER_RPM)
            )
        );

        // Right stick Y-axis: move the climber up and down with percentage output
        //Test: Push right stick up/down to move the climber. Releasing centers the stick and stops the climber.
        exampleMatchMode.whileTrue(
            Commands.run(() -> {
                double stickValue = MathUtil.applyDeadband(controller.getRightY(), CLIMBER_DEADBAND);
                climber.setClimberPercentage(stickValue);
            }, climber)
        );

        // Back button: move intake to stored/in position
        //Test: Press back button to pivot intake in to kPivotInSetpoint.
        exampleMatchMode.and(controller.back()).onTrue(
            new PivotIntake(intakePivot, Constants.kPivotInSetpoint)
        );

        
    }
}