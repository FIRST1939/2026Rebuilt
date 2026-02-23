package frc.robot.subsystems.shooter;

import edu.wpi.first.math.geometry.Pose2d;
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
        SHOOTER_MAP.put(1.5, new ShooterParams(2800.0, 0.05,  0.42));
        SHOOTER_MAP.put(2.0, new ShooterParams(3100.0, 0.09,  0.51));
        SHOOTER_MAP.put(2.5, new ShooterParams(3400.0, 0.10,  0.58));
        SHOOTER_MAP.put(3.0, new ShooterParams(3650.0, 0.11,  0.65));
        SHOOTER_MAP.put(3.5, new ShooterParams(3900.0, 0.12,  0.71));
        SHOOTER_MAP.put(4.0, new ShooterParams(4100.0, 0.13,  0.78));
        SHOOTER_MAP.put(4.5, new ShooterParams(4350.0, 0.14,  0.84));
        SHOOTER_MAP.put(5.0, new ShooterParams(4550.0, 0.15,  0.91));
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
            logSolution(zero, futurePosition, new Translation2d());
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

        Solution solution = new Solution(
                compensatedParams.rpm,
                compensatedParams.hoodPositionRotations);

        logSolution(solution, futurePosition, compensatedShotVector);
        return solution;
    }

    // ---- Logging -------------------------------------------------------- //

    private void logSolution(Solution s, Translation2d futurePos,
                             Translation2d shotVec) {

        Logger.recordOutput("ShooterSolution/FlywheelRPM", s.flywheelRPM);
        Logger.recordOutput("ShooterSolution/HoodPositionRot", s.hoodPositionRotations);
        Logger.recordOutput("ShooterSolution/CompensatedSpeedMps", shotVec.getNorm());
        Logger.recordOutput("ShooterSolution/FuturePosition",
                new double[] { futurePos.getX(), futurePos.getY() });
        Logger.recordOutput("ShooterSolution/ShotVector",
                new double[] { shotVec.getX(), shotVec.getY() });
    }
}
