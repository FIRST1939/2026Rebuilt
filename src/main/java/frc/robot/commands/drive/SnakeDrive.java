package frc.robot.commands.drive;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.subsystems.drive.Drive;
import frc.robot.util.Util;

public class SnakeDrive extends JoystickDriveAtAngle {
    
    private static final SlewRateLimiter xLimiter = new SlewRateLimiter(3.5);
    private static final SlewRateLimiter yLimiter = new SlewRateLimiter(3.5);

    public SnakeDrive(
        Drive drive, 
        DoubleSupplier xSupplier, 
        DoubleSupplier ySupplier, 
        DoubleSupplier omegaSupplier) {

            super(
                drive, 
                xSupplier, 
                ySupplier, 
                omegaSupplier, 
                () -> {

                    Translation2d robotVelocity = Util.getLinearVelocityFromJoysticks(xSupplier.getAsDouble(), ySupplier.getAsDouble());
                    double xComponent = xLimiter.calculate(robotVelocity.getX());
                    double yComponent = yLimiter.calculate(robotVelocity.getY());

                    if (xComponent == 0.0 && yComponent == 0.0) {

                        return drive.getRotation();
                    }

                    Rotation2d driverRelative = new Rotation2d(xComponent, yComponent);
                    return Util.isRedAlliance() ? driverRelative.plus(new Rotation2d(Math.PI)) : driverRelative;
                }
            );
    }

    @Override
    public void initialize() {

        super.initialize();

        ChassisSpeeds chassisSpeeds = super.m_drive.getRobotRelativeChassisSpeeds();
        double xPower = chassisSpeeds.vxMetersPerSecond / super.m_drive.getMaxLinearSpeedMetersPerSec();
        double yPower = chassisSpeeds.vyMetersPerSecond / super.m_drive.getMaxLinearSpeedMetersPerSec();

        if (Util.isRedAlliance()) {

            xPower *= -1;
            yPower *= -1;
        }

        xLimiter.reset(xPower);
        yLimiter.reset(yPower);
    }
}
