package frc.robot.commands.drive;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.drive.Drive;

public class DriveToPose extends Command {
    
    private final Drive m_drive;
    private final Pose2d m_pose;

    private final ProfiledPIDController m_xPidController = new ProfiledPIDController(
        1.0, 
        0.0, 
        0.0,
        new TrapezoidProfile.Constraints(2.0, 3.0)
    );

    private final ProfiledPIDController m_yPidController = new ProfiledPIDController(
        1.0, 
        0.0, 
        0.0,
        new TrapezoidProfile.Constraints(2.0, 3.0)
    );

    private final ProfiledPIDController m_headingPidController = new ProfiledPIDController(
        5.0,
        0.0,
        0.4,
        new TrapezoidProfile.Constraints(8.0, 20.0)
    );

    public DriveToPose (Drive drive, Pose2d pose) {

        m_drive = drive;
        m_pose = pose;

        m_headingPidController.enableContinuousInput(-Math.PI, Math.PI);

        addRequirements(drive);
    }

    @Override
    public void initialize () {

        m_xPidController.setGoal(m_pose.getX());
        m_yPidController.setGoal(m_pose.getY());
        m_headingPidController.setGoal(m_pose.getRotation().getRadians());
    }

    @Override
    public void execute () {

        Pose2d robotPose = m_drive.getPose();

        m_drive.runVelocity(ChassisSpeeds.fromFieldRelativeSpeeds(
            m_xPidController.calculate(robotPose.getX()),
            m_yPidController.calculate(robotPose.getY()),
            m_headingPidController.calculate(robotPose.getRotation().getRadians()),
            m_drive.getRotation()
        ));
    }

    @Override
    public boolean isFinished () {

        Translation2d translationError = m_pose.getTranslation().minus(m_drive.getPose().getTranslation());
        Rotation2d rotationError = m_pose.getRotation().minus(m_drive.getRotation());
        return translationError.getX() < 0.025 && translationError.getY() < 0.025 && rotationError.getDegrees() < 2.5;
    }

    @Override
    public void end (boolean interrupted) {

        m_drive.stop();
    }
}
