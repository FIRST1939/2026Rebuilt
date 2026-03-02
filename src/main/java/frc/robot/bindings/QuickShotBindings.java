package frc.robot.bindings;

import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import frc.robot.Constants;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.spindexer.Spindexer;
import frc.robot.subsystems.feeder.Feeder;
import frc.robot.subsystems.shooter.Shooter;


import frc.robot.commands.shooter.FollowShooterSetpoints;
import frc.robot.commands.shooter.RunFlywheelAndHood;
import frc.robot.commands.shooter.ShootSequence;
import frc.robot.commands.spindexer.RunSpindexerVelocity;
import frc.robot.commands.feeder.RunFeederVelocity;
import java.util.function.Supplier;

import frc.robot.commands.intake.RunPivotAndRollerAuto;
import frc.robot.subsystems.shooter.ShooterSolutionFinder;

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



    public static void configure(
            Trigger quickShotMode,
            CommandXboxController controller,
            Intake intake,
            Spindexer spindexer,
            Feeder feeder,
            Shooter shooter,
            Supplier<ShooterSolutionFinder> solutionFinderSupplier) {

         quickShotMode.and(controller.leftBumper()).toggleOnTrue(new FollowShooterSetpoints(shooter,
                 m_flywheelRPM::getAsDouble,
                 m_hoodPosition::getAsDouble)
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

        
        // Right trigger: pivot intake out and run intake roller while held
        //quickShotMode.and(controller.rightTrigger()).whileTrue(
        //    new RunPivotAndRoller(intake, Constants.kPivotOutSetpoint, () -> ROLLER_VELOCITY)
        //);

        // Right bumper: pivot intake to idle and run intake roller inward while held
        //quickShotMode.and(controller.rightBumper()).whileTrue(
        //    new RunPivotAndRoller(intake, kPivotIdleSetpoint, () -> ROLLER_VELOCITY_INWARD)
        //);

    
            // Y button: static shot — spin up flywheel + hood, run feeder and spindexer, all while held
        //Test: Hold Y to fire a static shot at fixed RPM/hood. Release to stop everything.
        quickShotMode.and(controller.y()).whileTrue(
            Commands.parallel(
                new RunFlywheelAndHood(shooter, () -> 3000, () -> 0.05)
                //new RunFeederVelocity(feeder, FEEDER_RPM),
                //new RunSpindexerVelocity(spindexer, SPINDEXER_RPM)
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
        quickShotMode.and(controller.start()).toggleOnTrue(
            new FollowShooterSetpoints(shooter,
                () -> solutionFinderSupplier.get().getLatestSolution().flywheelRPM,
                () -> solutionFinderSupplier.get().getLatestSolution().hoodPositionRotations)
            .finallyDo(() -> {
                shooter.setHoodPosition(0);
                shooter.setFlywheelPercentage(0);
            })
        );
  //update Drive.java  
//  /** Returns the measured chassis speeds of the robot. */
//  @AutoLogOutput(key = "SwerveChassisSpeeds/Measured")
//  public ChassisSpeeds getChassisSpeeds() {
//    return kinematics.toChassisSpeeds(getModuleStates());
//  }

//Udpate Robot.container
//
//import edu.wpi.first.math.geometry.Pose2d;
//import edu.wpi.first.math.kinematics.ChassisSpeeds;
//   public void updateShooterSolution() {
//         m_solutionFinder = m_solutionFinderSelector.get();

//         Pose2d pose = m_drive.getPose();
//         ChassisSpeeds speeds = m_drive.getChassisSpeeds();

//         m_solutionFinder.update(pose, speeds);

//         Logger.recordOutput("ShooterSolution/RobotPose", pose);
//         Logger.recordOutput("ShooterSolution/RobotPoseX", pose.getX());
//         Logger.recordOutput("ShooterSolution/RobotPoseY", pose.getY());
//         Logger.recordOutput("ShooterSolution/ChassisSpeedX", speeds.vxMetersPerSecond);
//         Logger.recordOutput("ShooterSolution/ChassisSpeedY", speeds.vyMetersPerSecond);
//     }


    }
}
