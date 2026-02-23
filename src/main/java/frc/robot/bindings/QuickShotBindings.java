package frc.robot.bindings;

import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.spindexer.Spindexer;
import frc.robot.subsystems.feeder.Feeder;
import frc.robot.subsystems.shooter.Shooter;

import frc.robot.commands.shooter.FollowShooterSetpoints;
import frc.robot.commands.shooter.ShootSequence;
import frc.robot.commands.intake.RunRoller;

public class QuickShotBindings {

    // Dashboard-tunable values for QuickShot testing.
    private static final LoggedNetworkNumber m_flywheelRPM =
            new LoggedNetworkNumber("/QuickShot/Flywheel RPM", 3000);
    private static final LoggedNetworkNumber m_hoodPosition =
            new LoggedNetworkNumber("/QuickShot/Hood Target Position", 0.05);

    // Increment constants
    private static final double RPM_INCREMENT = 50;
    private static final double HOOD_INCREMENT = 1.0 / 360.0; // 1 degree in rotations
    private static final double HOOD_MIN = 0.0;
    private static final double HOOD_MAX = 0.1;

    private static final double FEEDER_RPM = 2000;
    private static final double FEEDER_THRESHOLD = 500;
    private static final double SPINDEXER_RPM = 120;



    public static void configure(
            Trigger quickShotMode,
            CommandXboxController controller,
            Intake intake,
            Spindexer spindexer,
            Feeder feeder,
            Shooter shooter) {

        quickShotMode.and(controller.y()).toggleOnTrue(new FollowShooterSetpoints(shooter, m_flywheelRPM::getAsDouble, m_hoodPosition::getAsDouble)
        .finallyDo(() -> {
            shooter.setHoodPosition(0);
            shooter.setFlywheelPercentage(0);
        })
        );

        // Right bumper: increase RPM by 50
        quickShotMode.and(controller.rightBumper()).onTrue(
            Commands.runOnce(() -> m_flywheelRPM.set(m_flywheelRPM.getAsDouble() + RPM_INCREMENT))
        );

        // Left bumper: decrease RPM by 50
        quickShotMode.and(controller.leftBumper()).onTrue(
            Commands.runOnce(() -> m_flywheelRPM.set(m_flywheelRPM.getAsDouble() - RPM_INCREMENT))
        );

        // Right trigger: increase hood position by 1 degree, clamped to 0.1
        quickShotMode.and(controller.rightTrigger()).onTrue(
            Commands.runOnce(() -> m_hoodPosition.set(
                MathUtil.clamp(m_hoodPosition.getAsDouble() + HOOD_INCREMENT, HOOD_MIN, HOOD_MAX)))
        );

        // Left trigger: decrease hood position by 1 degree, clamped to 0.0
        quickShotMode.and(controller.leftTrigger()).onTrue(
            Commands.runOnce(() -> m_hoodPosition.set(
                MathUtil.clamp(m_hoodPosition.getAsDouble() - HOOD_INCREMENT, HOOD_MIN, HOOD_MAX)))
        );

        
        quickShotMode.and(controller.b()).whileTrue(new RunRoller(intake,() -> 1000));

          quickShotMode.and(controller.a()).whileTrue(
            ShootSequence.create(
                shooter, feeder, spindexer,
                FEEDER_RPM, FEEDER_THRESHOLD, SPINDEXER_RPM)
        );

    }
}
