package frc.robot.bindings;

import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import frc.robot.Constants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.spindexer.Spindexer;
import frc.robot.subsystems.feeder.Feeder;
import frc.robot.subsystems.shooter.Shooter;

import frc.robot.commands.drive.DriveCommands;
import frc.robot.commands.intake.RunPivotAndRoller;
import frc.robot.commands.intake.RunPivotAndRollerAuto;
import frc.robot.commands.intake.RunPivot;
import frc.robot.commands.shooter.FollowShooterSetpoints;
import frc.robot.commands.shooter.RunFlywheelAndHood;
import frc.robot.commands.shooter.ShootSequence;
import frc.robot.commands.spindexer.RunSpindexerVelocity;
import frc.robot.commands.feeder.RunFeederVelocity;

import frc.robot.subsystems.shooter.ShooterSolutionFinder;
import frc.robot.util.ShotSolver;
import frc.robot.util.Util;

public class QuickShotBindings {

    // Dashboard-tunable values for QuickShot testing.
    private static final LoggedNetworkNumber m_flywheelRPM =
            new LoggedNetworkNumber("/QuickShot/Flywheel RPM", 3000);
    private static final LoggedNetworkNumber m_hoodPosition =
            new LoggedNetworkNumber("/QuickShot/Hood Target Position", 0.05);

    // Increment constants
    private static final double RPM_INCREMENT = 50;
    private static final double HOOD_INCREMENT = 0.5 / 360.0; // 1 degree in rotations
    private static final double HOOD_MIN = 0.0;
    private static final double HOOD_MAX = 0.1;

    private static final double FEEDER_RPM = 2000;
    private static final double FEEDER_THRESHOLD = 500;
    private static final double SPINDEXER_RPM = 350;
    private static final double ROLLER_VELOCITY = -1500.0;
    private static final double ROLLER_VELOCITY_INWARD = 1000.0;
    private static final double kPivotIdleSetpoint = 0.4;
    private static final double DPAD_SLOW_SPEED = 0.5; // m/s for d-pad slow drive



    public static void configure(
            Trigger quickShotMode,
            CommandXboxController controller,
            CommandXboxController driverController,
            Drive drive,
            Intake intake,
            Spindexer spindexer,
            Feeder feeder,
            Shooter shooter,
            ShotSolver shotSolver) {

         quickShotMode.and(controller.x()).toggleOnTrue(new FollowShooterSetpoints(shooter,
                 m_flywheelRPM::getAsDouble,
                 m_hoodPosition::getAsDouble)
        .finallyDo(() -> {
            shooter.setHoodPosition(0);
            shooter.setFlywheelPercentage(0);
        })
        );

        // Left bumper: follow ShotSolver solution for testing
        quickShotMode.and(controller.leftBumper()).toggleOnTrue(new FollowShooterSetpoints(shooter,
                () -> shotSolver.getShotSolution().flywheelRPM,
                () -> shotSolver.getShotSolution().hoodPositionRotations)
        .finallyDo(() -> {
            shooter.setHoodPosition(0);
            shooter.setFlywheelPercentage(0);
        })
        );

        

        quickShotMode.and(controller.leftTrigger()).whileTrue(
            ShootSequence.create(
                shooter, feeder, spindexer,
                FEEDER_RPM, FEEDER_THRESHOLD, SPINDEXER_RPM)
        );
 


        // Right bumper: increase RPM by 50
        quickShotMode.and(controller.povRight()).onTrue(
            Commands.runOnce(() -> m_flywheelRPM.set(m_flywheelRPM.getAsDouble() + RPM_INCREMENT))
        );

        // Left bumper: decrease RPM by 50
        quickShotMode.and(controller.povLeft()).onTrue(
            Commands.runOnce(() -> m_flywheelRPM.set(m_flywheelRPM.getAsDouble() - RPM_INCREMENT))
        );

        // Right trigger: increase hood position by 1 degree, clamped to 0.1
        quickShotMode.and(controller.povUp()).onTrue(
            Commands.runOnce(() -> m_hoodPosition.set(
                MathUtil.clamp(m_hoodPosition.getAsDouble() + HOOD_INCREMENT, HOOD_MIN, HOOD_MAX)))
        );

        // Left trigger: decrease hood position by 1 degree, clamped to 0.0
        quickShotMode.and(controller.povDown()).onTrue(
            Commands.runOnce(() -> m_hoodPosition.set(
                MathUtil.clamp(m_hoodPosition.getAsDouble() - HOOD_INCREMENT, HOOD_MIN, HOOD_MAX)))
        );

        // --- Operator intake bindings ---

        // A button: deploy intake — pivot out, run roller, and spindexer while held
        quickShotMode.and(controller.a()).whileTrue(
            new RunPivotAndRoller(intake, Constants.kPivotOutSetpoint, () -> Constants.kBaseRollerIntakeVelocity)
                .alongWith(new RunSpindexerVelocity(spindexer, SPINDEXER_RPM))
        );

        // B button: pivot intake back to idle position
        quickShotMode.and(controller.b()).onTrue(
            new RunPivot(intake, Constants.kPivotIdleSetpoint)
        );
    
        // Right bumper: increase lookahead time by 0.01
        quickShotMode.and(controller.rightBumper()).onTrue(
            Commands.runOnce(() -> shotSolver.increaseLookahead())
        );

        // A button: decrease lookahead time by 0.01
        quickShotMode.and(controller.rightTrigger()).onTrue(
            Commands.runOnce(() -> shotSolver.decreaseLookahead())
        );

            // Y button: static shot — spin up flywheel + hood, run feeder and spindexer, all while held
        //Test: Hold Y to fire a static shot at fixed RPM/hood. Release to stop everything.
        quickShotMode.and(controller.y()).whileTrue(
            Commands.parallel(
                new RunFlywheelAndHood(shooter, () -> 3000, () -> 0.05),
                new RunFeederVelocity(feeder, FEEDER_RPM),
                new RunSpindexerVelocity(spindexer, SPINDEXER_RPM)
            )
        );

        // A button: static shot — spin up flywheel + hood, run feeder and spindexer, all while held
        //Test: Hold A to fire.
        //quickShotMode.and(controller.a()).whileTrue(
        //    Commands.parallel(
        //        new RunFeederVelocity(feeder, FEEDER_RPM),
        //        new RunSpindexerVelocity(spindexer, SPINDEXER_RPM)
        //    )
        //);

        // Start button: follow interpolating shooter solution for tuning
        // quickShotMode.and(controller.start()).toggleOnTrue(
        //     new FollowShooterSetpoints(shooter,
        //         () -> solutionFinderSupplier.get().getLatestSolution().flywheelRPM,
        //         () -> solutionFinderSupplier.get().getLatestSolution().hoodPositionRotations)
        //     .finallyDo(() -> {
        //         shooter.setHoodPosition(0);
        //         shooter.setFlywheelPercentage(0);
        //     })
        // );

        // --- Driver joystick bindings ---

        // Right trigger: heading lock to shot solver aim heading
        quickShotMode.and(driverController.rightTrigger()).onTrue(
            DriveCommands.joystickDriveAtAngle(
                drive,
                () -> -driverController.getLeftY(),
                () -> -driverController.getLeftX(),
                () -> -driverController.getRightX(),
                () -> shotSolver.getShotSolution().aimHeading
            )
        );

        
        quickShotMode.and(driverController.a()).onTrue(
            DriveCommands.joystickDriveAtAngle(
                drive,
                () -> -driverController.getLeftY(),
                () -> -driverController.getLeftX(),
                () -> -driverController.getRightX(),
                () -> Util.isRedAlliance() ? new Rotation2d() : new Rotation2d(Math.PI)
            )
        );

        
        quickShotMode.and(driverController.y()).onTrue(
            DriveCommands.joystickDriveAtAngle(
                drive,
                () -> -driverController.getLeftY(),
                () -> -driverController.getLeftX(),
                () -> -driverController.getRightX(),
                () -> Util.isRedAlliance() ? new Rotation2d(Math.PI) : new Rotation2d()
            )
        );

        // X button: stop with X
        quickShotMode.and(driverController.x()).onTrue(Commands.runOnce(drive::stopWithX, drive));

        // D-pad: drive slowly in the pressed direction (field-relative) while aiming at the goal
        quickShotMode.and(driverController.povUp()).whileTrue(
            DriveCommands.joystickDriveAtAngle(
                drive,
                () -> DPAD_SLOW_SPEED / drive.getMaxLinearSpeedMetersPerSec(),
                () -> 0,
                () -> 0,
                () -> shotSolver.getShotSolution().aimHeading
            )
        );

        quickShotMode.and(driverController.povDown()).whileTrue(
            DriveCommands.joystickDriveAtAngle(
                drive,
                () -> -DPAD_SLOW_SPEED / drive.getMaxLinearSpeedMetersPerSec(),
                () -> 0,
                () -> 0,
                () -> shotSolver.getShotSolution().aimHeading
            )
        );

        quickShotMode.and(driverController.povLeft()).whileTrue(
            DriveCommands.joystickDriveAtAngle(
                drive,
                () -> 0,
                () -> DPAD_SLOW_SPEED / drive.getMaxLinearSpeedMetersPerSec(),
                () -> 0,
                () -> shotSolver.getShotSolution().aimHeading
            )
        );

        quickShotMode.and(driverController.povRight()).whileTrue(
            DriveCommands.joystickDriveAtAngle(
                drive,
                () -> 0,
                () -> -DPAD_SLOW_SPEED / drive.getMaxLinearSpeedMetersPerSec(),
                () -> 0,
                () -> shotSolver.getShotSolution().aimHeading
            )
        );

    }
}
