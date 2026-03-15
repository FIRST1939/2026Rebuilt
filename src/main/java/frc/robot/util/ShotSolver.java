package frc.robot.util;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.interpolation.InterpolatingTreeMap;
import edu.wpi.first.math.interpolation.Interpolator;
import edu.wpi.first.math.interpolation.InverseInterpolator;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.Constants;
import frc.robot.subsystems.shooter.ShooterConstants;

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
        public double timeOfFlight;

        public ShotSolution(double flywheelRPM, double hoodPositionRotations, Rotation2d aimHeading, double timeOfFlight) {

            this.flywheelRPM = flywheelRPM;
            this.hoodPositionRotations = hoodPositionRotations;
            this.aimHeading = aimHeading;
            this.timeOfFlight = timeOfFlight;
        }
    }

    private static final InterpolatingTreeMap<Double, ShooterParams> kShooterMap =new InterpolatingTreeMap<>(
        InverseInterpolator.forDouble(), 
        ShooterParams.kInterpolator
    );

    static {

        kShooterMap.put(1.09, new ShooterParams(2750, 0.0125, 1.18));
        kShooterMap.put(1.5, new ShooterParams(2850, 0.0264, 1.33));
        kShooterMap.put(2.0, new ShooterParams(2650, 0.0458, 1.145));
        kShooterMap.put(2.5, new ShooterParams(2850, 0.0458, 1.175));
        kShooterMap.put(3.0, new ShooterParams(3000, 0.04861, 1.3));
        kShooterMap.put(3.5, new ShooterParams(3300, 0.05277, 1.36));
        kShooterMap.put(4.0, new ShooterParams(3400, 0.05694, 1.45));
        kShooterMap.put(4.5, new ShooterParams(3500, 0.0597, 1.5));
        kShooterMap.put(5.0, new ShooterParams(3600, 0.0625, 1.55));
    }

    private ShotSolution m_shotSolution;


    public void calculateShotSolution(Pose2d measuredRobotPose, ChassisSpeeds robotSpeeds) {

        // Get the shooter pose from the robot pose.
        Pose2d shooterPose = getShooterPose(measuredRobotPose);

        ShotSolution staticShotSolution = calculateShotSolutionStatic(shooterPose);
        Logger.recordOutput("ShotSolver/Static/FlywheelRPM", staticShotSolution.flywheelRPM);
        Logger.recordOutput("ShotSolver/Static/HoodPosition", staticShotSolution.hoodPositionRotations);
        Logger.recordOutput("ShotSolver/Static/AimHeading", staticShotSolution.aimHeading.getDegrees());
        Logger.recordOutput("ShotSolver/Static/TimeOfFlight", staticShotSolution.timeOfFlight);

        // Calculate a look-ahead shooter pose by projecting the shooter position forward in time by a small amount.
        Pose2d lookAheadShooterPose = findFuturePose(shooterPose, robotSpeeds, Constants.kLookAheadTime);
        ShotSolution lookAheadShooterSolution = calculateShotSolutionStatic(lookAheadShooterPose);
        double lookAheadTimeOfFlight = lookAheadShooterSolution.timeOfFlight;
        Logger.recordOutput("ShotSolver/LookAhead/FlywheelRPM", lookAheadShooterSolution.flywheelRPM);
        Logger.recordOutput("ShotSolver/LookAhead/HoodPosition", lookAheadShooterSolution.hoodPositionRotations);
        Logger.recordOutput("ShotSolver/LookAhead/AimHeading", lookAheadShooterSolution.aimHeading.getDegrees());
        Logger.recordOutput("ShotSolver/LookAhead/TimeOfFlight", lookAheadTimeOfFlight);
        Logger.recordOutput("ShotSolver/LookAhead/Pose", lookAheadShooterPose);

        // Get future shooter pose by projecting the shooter position forward in time by the look-ahead time of flight
        Pose2d futureShooterPose = findFuturePose(shooterPose, robotSpeeds, lookAheadTimeOfFlight);
        ShotSolution futureShooterSolution = calculateShotSolutionStatic(futureShooterPose);
        Logger.recordOutput("ShotSolver/Future/FlywheelRPM", futureShooterSolution.flywheelRPM);
        Logger.recordOutput("ShotSolver/Future/HoodPosition", futureShooterSolution.hoodPositionRotations);
        Logger.recordOutput("ShotSolver/Future/AimHeading", futureShooterSolution.aimHeading.getDegrees());
        Logger.recordOutput("ShotSolver/Future/TimeOfFlight", futureShooterSolution.timeOfFlight);
        Logger.recordOutput("ShotSolver/Future/Pose", futureShooterPose);

        m_shotSolution = futureShooterSolution;
    }

    public ShotSolution getShotSolution () {

        return m_shotSolution;
    }

    /**
     * Calculate a shot solution from a static pose (no velocity compensation).
     * Useful for auto pre-computation or anywhere you just have a position.
     */
    public ShotSolution calculateShotSolutionStatic(Pose2d robotPose) {

        Translation2d robotPosition = robotPose.getTranslation();
        Translation2d robotToTarget = Util.getHubPosition().minus(robotPosition);

        double distanceToTarget = robotToTarget.getNorm();
        ShooterParams params = kShooterMap.get(distanceToTarget);
        Rotation2d aimHeading = robotToTarget.getAngle().plus(new Rotation2d(Math.PI));

        return new ShotSolution(
            params.kFlywheelRPM,
            params.kHoodPositionRotations,
            aimHeading,
            params.kTimeOfFlight
        );
    }

    public Pose2d findFuturePose(Pose2d robotPose, ChassisSpeeds robotSpeeds, double lookaheadTime) {
    
        Pose2d futureRobotPose = robotPose.exp(
            new Twist2d(
                robotSpeeds.vxMetersPerSecond * lookaheadTime,
                robotSpeeds.vyMetersPerSecond * lookaheadTime,
                robotSpeeds.omegaRadiansPerSecond * lookaheadTime
            )
        );

        return futureRobotPose;
    };
 
     /**
     * Get the shooter position on the field from a robot pose.
     */
    public static Pose2d getShooterPose(Pose2d robotPose) {

        return robotPose.transformBy(
            new Transform2d(
                ShooterConstants.kRobotToShooter,
                new Rotation2d()
            )
        );
    }

}