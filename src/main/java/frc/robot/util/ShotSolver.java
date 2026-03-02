package frc.robot.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingTreeMap;
import edu.wpi.first.math.interpolation.Interpolator;
import edu.wpi.first.math.interpolation.InverseInterpolator;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

public class ShotSolver {
    
    private static class ShooterParams {

        public final double kFlywheelRPM;
        public final double kHoodPositionRotations;
        public final double kTimeOfFlight;

        public ShooterParams(double flywheelRPM, double hoodPositionRotations, double timeOfFlight) {

            kFlywheelRPM = flywheelRPM;
            kHoodPositionRotations = hoodPositionRotations;
            kTimeOfFlight = timeOfFlight;
        }

        public static final Interpolator<ShooterParams> kInterpolator =
            (startValue, endValue, t) -> new ShooterParams(
                startValue.kFlywheelRPM + (endValue.kFlywheelRPM - startValue.kFlywheelRPM) * t,
                startValue.kHoodPositionRotations + (endValue.kHoodPositionRotations - startValue.kHoodPositionRotations) * t,
                startValue.kTimeOfFlight + (endValue.kTimeOfFlight - startValue.kTimeOfFlight) * t
            );
    }

    public static class ShotSolution {

        public double flywheelRPM;
        public double hoodPositionRotations;
        public Rotation2d aimHeading;

        public ShotSolution(double flywheelRPM, double hoodPositionRotations, Rotation2d aimHeading) {

            this.flywheelRPM = flywheelRPM;
            this.hoodPositionRotations = hoodPositionRotations;
            this.aimHeading = aimHeading;
        }
    }

    private static final InterpolatingTreeMap<Double, ShooterParams> kShooterMap =new InterpolatingTreeMap<>(
        InverseInterpolator.forDouble(), 
        ShooterParams.kInterpolator
    );

    static {

        kShooterMap.put(1.09, new ShooterParams(2750, 0.0125, 1.18));
        kShooterMap.put(1.93, new ShooterParams(2650, 0.0458, 1.14));
        kShooterMap.put(2.94, new ShooterParams(3000, 0.047, 1.31));
        kShooterMap.put(3.54, new ShooterParams(3250, 0.0458, 1.42));
        kShooterMap.put(5.70, new ShooterParams(3400, 0.07, 1.34));
    }

    private ShotSolution m_shotSolution;

    public void calculateShotSolution (Pose2d robotPose, ChassisSpeeds robotSpeeds) {

        Translation2d robotVelocity = new Translation2d(robotSpeeds.vxMetersPerSecond, robotSpeeds.vyMetersPerSecond);

        Translation2d futurePosition = robotPose.getTranslation().plus(robotVelocity.times(0.1));

        Translation2d targetVector = Util.getHubPosition().minus(futurePosition);
        double rawDistance = targetVector.getNorm();

        if (rawDistance < 1e-3) {

            ShotSolution zero = new ShotSolution(0, 0, new Rotation2d());
            m_shotSolution = zero;
            return;
        }

        // Get info from map based on raw distance.
        ShooterParams rawParams = kShooterMap.get(rawDistance);

        double idealHorizontalSpeed = rawDistance / rawParams.kTimeOfFlight;

        Translation2d idealShotDirection = targetVector.div(rawDistance);
        Translation2d idealShotVector = idealShotDirection.times(idealHorizontalSpeed);
        Translation2d compensatedShotVector = idealShotVector.minus(robotVelocity);

        double compensatedSpeed = compensatedShotVector.getNorm();
        double virtualDistance = rawDistance * (compensatedSpeed / idealHorizontalSpeed);

        ShooterParams compensatedParams = kShooterMap.get(virtualDistance);

        // The compensated shot vector direction is where the robot needs to face
        Rotation2d aimHeading = compensatedShotVector.getAngle();

        ShotSolution solution = new ShotSolution(
            compensatedParams.kFlywheelRPM,
            compensatedParams.kHoodPositionRotations,
            aimHeading
        );

        m_shotSolution = solution;
    }

    public ShotSolution getShotSolution () {

        return m_shotSolution;
    }
}
