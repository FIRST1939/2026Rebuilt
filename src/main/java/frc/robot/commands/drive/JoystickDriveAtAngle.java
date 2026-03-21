package frc.robot.commands.drive;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.*;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.DriveConstants;
import frc.robot.util.Util;

public class JoystickDriveAtAngle extends Command {
    
    protected final Drive m_drive;
    private final DoubleSupplier m_xSupplier;
    private final DoubleSupplier m_ySupplier;
    private final DoubleSupplier m_omegaSupplier;

    private final Supplier<Rotation2d> m_rotationSupplier;
    private final ProfiledPIDController m_angleController;

    public JoystickDriveAtAngle(
        Drive drive, 
        DoubleSupplier xSupplier, 
        DoubleSupplier ySupplier, 
        DoubleSupplier omegaSupplier,
        Supplier<Rotation2d> rotationSupplier) {

            m_drive = drive;
            m_xSupplier = xSupplier;
            m_ySupplier = ySupplier;
            m_omegaSupplier = omegaSupplier;
            m_rotationSupplier = rotationSupplier;

            m_angleController = new ProfiledPIDController(
                DriveConstants.kAngleControllerP,
                DriveConstants.kAngleControllerI,
                DriveConstants.kAngleControllerD,
                new TrapezoidProfile.Constraints(
                    DriveConstants.kAngleControllerMaxVelocity, 
                    DriveConstants.kAngleControllerMaxAcceleration
                )
            );

            m_angleController.enableContinuousInput(-Math.PI, Math.PI);

            addRequirements(drive);
    }

    @Override
    public void initialize() {

        m_angleController.reset(m_drive.getRotation().getRadians());
    }

    @Override
    public void execute() {

        // Get linear velocity
        Translation2d linearVelocity = Util.getLinearVelocityFromJoysticks(m_xSupplier.getAsDouble(), m_ySupplier.getAsDouble());

        // Calculate angular speed
        double omega = m_angleController.calculate(m_drive.getRotation().getRadians(), m_rotationSupplier.get().getRadians());

        // Convert to field relative speeds & send command
        ChassisSpeeds speeds = new ChassisSpeeds(
            linearVelocity.getX() * m_drive.getMaxLinearSpeedMetersPerSec(),
            linearVelocity.getY() * m_drive.getMaxLinearSpeedMetersPerSec(),
            omega
        );

        m_drive.runVelocity(
            ChassisSpeeds.fromFieldRelativeSpeeds(
                speeds,
                Util.isRedAlliance() ? m_drive.getRotation().plus(new Rotation2d(Math.PI)) : m_drive.getRotation()
            )
        );
    }

    @Override
    public boolean isFinished() {

        return MathUtil.applyDeadband(m_omegaSupplier.getAsDouble(), ControllerConstants.kJoystickDeadband) != 0.0;
    }
}
