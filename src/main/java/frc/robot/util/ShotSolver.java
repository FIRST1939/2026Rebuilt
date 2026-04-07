package frc.robot.util;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingTreeMap;
import edu.wpi.first.math.interpolation.InverseInterpolator;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.subsystems.shooter.ShooterConstants;
import frc.robot.util.MapUtil.ShooterParams;
import frc.robot.util.MapUtil.ShotSolution;

public class ShotSolver {

    private static final InterpolatingTreeMap<Double, ShooterParams> kShooterMap = new InterpolatingTreeMap<>(
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
        kShooterMap.put(5.5, new ShooterParams(3700, 0.0654, 1.6));
        kShooterMap.put(6.0, new ShooterParams(3800, 0.0683, 1.65));
        kShooterMap.put(7.0, new ShooterParams(4000, 0.071, 1.7));
    }

    private ShotSolution m_shotSolution = new ShotSolution(0, 0, new Rotation2d(), 0);

    public ShotSolution getPPShotSolution(Pose2d robotPose) {

        return calculateShotSolutionStatic(robotPose, false);
    }

    public void calculateShotSolution(Pose2d measuredRobotPose, ChassisSpeeds robotSpeeds) {

        ShotSolution staticShotSolution = calculateShotSolutionStatic(measuredRobotPose, true);
        Logger.recordOutput("ShotSolver/Static/FlywheelRPM", staticShotSolution.flywheelRPM);
        Logger.recordOutput("ShotSolver/Static/HoodPosition", staticShotSolution.hoodPositionRotations);
        Logger.recordOutput("ShotSolver/Static/AimHeading", staticShotSolution.aimHeading.getDegrees());
        Logger.recordOutput("ShotSolver/Static/TimeOfFlight", staticShotSolution.timeOfFlight);

        Pose2d futureRobotPose = findFuturePose(measuredRobotPose, robotSpeeds);
        ShotSolution futureShooterSolution = calculateShotSolutionStatic(futureRobotPose, true);
        Logger.recordOutput("ShotSolver/Future/RobotPose", futureRobotPose);
        Logger.recordOutput("ShotSolver/Future/FlywheelRPM", futureShooterSolution.flywheelRPM);
        Logger.recordOutput("ShotSolver/Future/HoodPosition", futureShooterSolution.hoodPositionRotations);
        Logger.recordOutput("ShotSolver/Future/AimHeading", futureShooterSolution.aimHeading.getDegrees());
        Logger.recordOutput("ShotSolver/Future/TimeOfFlight", futureShooterSolution.timeOfFlight);

        m_shotSolution = futureShooterSolution;        
    }

    public ShotSolution getShotSolution () {

        return m_shotSolution;
    }

    /**
     * Calculate a shot solution from a static pose (no velocity compensation).
     * Distance/RPM/hood are based on the shooter position (where the ball leaves).
     * Aim heading is based on the robot center (what the robot actually rotates around).
     */
    public ShotSolution calculateShotSolutionStatic(Pose2d robotPose, boolean useFlipLogic) {

        // Distance from shooter to hub — used for RPM, hood, and time of flight lookup
        Translation2d shooterToTarget = Util.getHubPosition(useFlipLogic).minus(getShooterPose(robotPose).getTranslation());
        double distanceToTarget = shooterToTarget.getNorm();
        ShooterParams params = kShooterMap.get(distanceToTarget);

        return new ShotSolution(
            params.kFlywheelRPM,
            params.kHoodPositionRotations,
            getAimHeading(robotPose),
            params.kTimeOfFlight
        );
    }

    public Pose2d findFuturePose(Pose2d originalRobotPose, ChassisSpeeds chassisSpeeds) {

        Pose2d futurePose = originalRobotPose;

        for (int i = 0; i < 5; i++) {

            ShotSolution futureShotSolution = calculateShotSolutionStatic(futurePose, true);
            double futureTimeOfFlight = futureShotSolution.timeOfFlight;
            Rotation2d futureAimHeading = getAimHeading(futurePose);

            futurePose = new Pose2d(
                originalRobotPose.getTranslation().plus(
                    new Translation2d(
                        chassisSpeeds.vxMetersPerSecond * futureTimeOfFlight,
                        chassisSpeeds.vyMetersPerSecond * futureTimeOfFlight
                    )
                ),
                futureAimHeading
            );
        }

        return futurePose;
    }

    public static Rotation2d getAimHeading(Pose2d robotPose) {

        Translation2d shooterToTarget = Util.getHubPosition().minus(getShooterPose(robotPose).getTranslation());
        return shooterToTarget.getAngle().plus(new Rotation2d(Math.PI));
    }
 
    public static Pose2d getShooterPose(Pose2d robotPose) {

        return robotPose.transformBy(
            new Transform2d(
                ShooterConstants.kRobotToShooter,
                new Rotation2d()
            )
        );
    }
}
