package frc.robot.subsystems.shooter;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingTreeMap;
import edu.wpi.first.math.interpolation.Interpolator;
import edu.wpi.first.math.interpolation.InverseInterpolator;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;



// Based on https://blog.eeshwark.com/blog/shooting-on-the-fly

public class InterpolatingShooterSolutionFinder implements ShooterSolutionFinder {

    private static class ShooterParams {

        public final double rpm;
        public final double hoodPositionRotations;
        public final double timeOfFlight;

        public ShooterParams(double rpm, double hoodPositionRotations, double timeOfFlight) {
            this.rpm = rpm;
            this.hoodPositionRotations = hoodPositionRotations;
            this.timeOfFlight = timeOfFlight;
        }

        public static final Interpolator<ShooterParams> kInterpolator =
                (startValue, endValue, t) -> new ShooterParams(
                        startValue.rpm + (endValue.rpm - startValue.rpm) * t,
                        startValue.hoodPositionRotations
                                + (endValue.hoodPositionRotations - startValue.hoodPositionRotations) * t,
                        startValue.timeOfFlight
                                + (endValue.timeOfFlight - startValue.timeOfFlight) * t);
    }

    private final LoggedNetworkNumber m_goalPosX =
            new LoggedNetworkNumber("/ShooterSolution/Goal Pos X", 8.23);
    private final LoggedNetworkNumber m_goalPosY =
            new LoggedNetworkNumber("/ShooterSolution/Goal Pos Y", 4.11);
    private final LoggedNetworkNumber m_latencyCompensation =
            new LoggedNetworkNumber("/ShooterSolution/Latency Compensation", 0.1);

    private static final InterpolatingTreeMap<Double, ShooterParams> SHOOTER_MAP =
            new InterpolatingTreeMap<>(InverseInterpolator.forDouble(),
                                      ShooterParams.kInterpolator);

    static {
        SHOOTER_MAP.put(0.5, new ShooterParams(2000.0, 0.12,  0.42));
        SHOOTER_MAP.put(20.0, new ShooterParams(3100.0, 0.09,  0.51));
        SHOOTER_MAP.put(50.0, new ShooterParams(5400.0, 0.18,  0.75));

    }

    private Solution m_latestSolution = new Solution(0, 0);

    @Override
    public Solution update(Pose2d robotPose, ChassisSpeeds robotSpeeds) {

        Translation2d goalPosition = new Translation2d(
                m_goalPosX.getAsDouble(), m_goalPosY.getAsDouble());
        double latency = m_latencyCompensation.getAsDouble();

        m_latestSolution = calculate(robotPose, robotSpeeds,
                                     goalPosition, latency);
        return m_latestSolution;
    }

    @Override
    public Solution getLatestSolution() {
        return m_latestSolution;
    }

    private Solution calculate(Pose2d robotPose,
                               ChassisSpeeds robotSpeeds,
                               Translation2d goalPosition,
                               double latencyCompensation) {

        Translation2d robotVelocity = new Translation2d(
                robotSpeeds.vxMetersPerSecond, robotSpeeds.vyMetersPerSecond);

        Translation2d futurePosition =
                robotPose.getTranslation()
                        .plus(robotVelocity.times(latencyCompensation));

        Translation2d targetVector = goalPosition.minus(futurePosition);
        double rawDistance = targetVector.getNorm();

        if (rawDistance < 1e-3) {
            Solution zero = new Solution(0, 0);
            logSolution(zero, futurePosition, new Translation2d(), 0.0);
            return zero;
        }

        // Get info from map based on raw distance.
        ShooterParams rawParams = SHOOTER_MAP.get(rawDistance);

        double idealHorizontalSpeed = rawDistance / rawParams.timeOfFlight;

        Translation2d idealShotDirection = targetVector.div(rawDistance);
        Translation2d idealShotVector = idealShotDirection.times(idealHorizontalSpeed);
        Translation2d compensatedShotVector = idealShotVector.minus(robotVelocity);

        double compensatedSpeed = compensatedShotVector.getNorm();
        double virtualDistance = rawDistance * (compensatedSpeed / idealHorizontalSpeed);

        ShooterParams compensatedParams = SHOOTER_MAP.get(virtualDistance);

        // The compensated shot vector direction is where the robot needs to face
        Rotation2d aimHeading = compensatedShotVector.getAngle();

        Solution solution = new Solution(
                compensatedParams.rpm,
                compensatedParams.hoodPositionRotations,
                aimHeading);

        logSolution(solution, futurePosition, compensatedShotVector, idealHorizontalSpeed);
        return solution;
    }

    // ---- Logging -------------------------------------------------------- //

    private void logSolution(Solution s, Translation2d futurePos,
                             Translation2d shotVec, double idealHorizontalSpeed) {

        Logger.recordOutput("ShooterSolution/FlywheelRPM", s.flywheelRPM);
        Logger.recordOutput("ShooterSolution/HoodPositionRot", s.hoodPositionRotations);
        Logger.recordOutput("ShooterSolution/AimHeadingDeg", s.robotHeading.getDegrees());
        Logger.recordOutput("ShooterSolution/CompensatedSpeedMps", shotVec.getNorm());
        Logger.recordOutput("ShooterSolution/FuturePosition",
                new double[] { futurePos.getX(), futurePos.getY() });
        Logger.recordOutput("ShooterSolution/ShotVector",
                new double[] { shotVec.getX(), shotVec.getY() });
        Logger.recordOutput("ShooterSolution/IdealHorizontalSpeed", idealHorizontalSpeed);
    }
}
