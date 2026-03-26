package frc.robot.commands.drive;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.*;
import frc.robot.subsystems.drive.Drive;
import frc.robot.util.Util;

public class JoystickDrive extends Command {
    
    private final Drive m_drive;
    private final DoubleSupplier m_xSupplier;
    private final DoubleSupplier m_ySupplier;
    private final DoubleSupplier m_omegaSupplier;

    public JoystickDrive(
        Drive drive, 
        DoubleSupplier xSupplier, 
        DoubleSupplier ySupplier, 
        DoubleSupplier omegaSupplier) {

            m_drive = drive;
            m_xSupplier = xSupplier;
            m_ySupplier = ySupplier;
            m_omegaSupplier = omegaSupplier;

            addRequirements(drive);
    }

    @Override
    public void execute() {

        // Get linear velocity
        Translation2d linearVelocity = Util.getLinearVelocityFromJoysticks(m_xSupplier.getAsDouble(), m_ySupplier.getAsDouble());

        // Apply rotation deadband
        double omega = MathUtil.applyDeadband(m_omegaSupplier.getAsDouble(), ControllerConstants.kJoystickDeadband);

        // Square rotation value for more precise control
        omega = Math.copySign(omega * omega, omega);

        // Convert to field relative speeds & send command
        ChassisSpeeds speeds = new ChassisSpeeds(
            linearVelocity.getX() * m_drive.getMaxLinearSpeedMetersPerSec(),
            linearVelocity.getY() * m_drive.getMaxLinearSpeedMetersPerSec(),
            omega * m_drive.getMaxAngularSpeedRadPerSec()
        );

        m_drive.runVelocity(
            ChassisSpeeds.fromFieldRelativeSpeeds(
                speeds,
                Util.isRedAlliance() ? m_drive.getRotation().plus(new Rotation2d(Math.PI)) : m_drive.getRotation()
            )
        );
    }
}
