package frc.robot.util;

import java.util.Arrays;
import java.util.List;

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

public class FerrySolver {
    
    private static final InterpolatingTreeMap<Double, ShooterParams> kFerryMap = new InterpolatingTreeMap<>(
        InverseInterpolator.forDouble(), 
        ShooterParams.kInterpolator
    );

    private static final List<Translation2d> kBlueFerryTargets = Arrays.asList(
        new Translation2d(FieldConstants.LinesVertical.allianceZone, FieldConstants.LinesHorizontal.leftBumpMiddle),
        new Translation2d(FieldConstants.LinesVertical.allianceZone, FieldConstants.LinesHorizontal.rightBumpMiddle)
    );

    private static final List<Translation2d> kRedFerryTargets = Arrays.asList(
        new Translation2d(FieldConstants.LinesVertical.oppAllianceZone, FieldConstants.LinesHorizontal.leftBumpMiddle),
        new Translation2d(FieldConstants.LinesVertical.oppAllianceZone, FieldConstants.LinesHorizontal.rightBumpMiddle)
    );

    static {

        kFerryMap.put(0.0, new ShooterParams(2750, 0.071, 0.0));
        kFerryMap.put(7.0, new ShooterParams(2750, 0.071, 0.0));
    }

    private ShotSolution m_ferrySolution = new ShotSolution(0, 0, new Rotation2d(), 0);

    public void calculateFerrySolution(Pose2d measuredRobotPose, ChassisSpeeds robotSpeeds) {

        ShotSolution staticFerrySolution = calculateFerrySolutionStatic(measuredRobotPose);
        Logger.recordOutput("FerrySolver/Static/FlywheelRPM", staticFerrySolution.flywheelRPM);
        Logger.recordOutput("FerrySolver/Static/HoodPosition", staticFerrySolution.hoodPositionRotations);
        Logger.recordOutput("FerrySolver/Static/AimHeading", staticFerrySolution.aimHeading.getDegrees());
        Logger.recordOutput("FerrySolver/Static/TimeOfFlight", staticFerrySolution.timeOfFlight);

        m_ferrySolution = staticFerrySolution;

        /*
        Pose2d futureRobotPose = findFuturePose(measuredRobotPose, robotSpeeds);
        ShotSolution futureShooterSolution = calculateShotSolutionStatic(futureRobotPose);
        Logger.recordOutput("ShotSolver/Future/RobotPose", futureRobotPose);
        Logger.recordOutput("ShotSolver/Future/FlywheelRPM", futureShooterSolution.flywheelRPM);
        Logger.recordOutput("ShotSolver/Future/HoodPosition", futureShooterSolution.hoodPositionRotations);
        Logger.recordOutput("ShotSolver/Future/AimHeading", futureShooterSolution.aimHeading.getDegrees());
        Logger.recordOutput("ShotSolver/Future/TimeOfFlight", futureShooterSolution.timeOfFlight);

        m_ferrySolution = futureShooterSolution;
        */ 
    }

    public ShotSolution getFerrySolution () {

        return m_ferrySolution;
    }

    /**
     * Calculate a shot solution from a static pose (no velocity compensation).
     * Distance/RPM/hood are based on the shooter position (where the ball leaves).
     * Aim heading is based on the robot center (what the robot actually rotates around).
     */
    public ShotSolution calculateFerrySolutionStatic(Pose2d robotPose) {

        List<Translation2d> ferryTargets = Util.isRedAlliance() ? kRedFerryTargets : kBlueFerryTargets;
        Translation2d ferryTarget = robotPose.getTranslation().nearest(ferryTargets);

        Translation2d shooterToTarget = ferryTarget.minus(getShooterPose(robotPose).getTranslation());
        double distanceToTarget = shooterToTarget.getNorm();
        ShooterParams params = kFerryMap.get(distanceToTarget);

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

            ShotSolution futureShotSolution = calculateFerrySolutionStatic(futurePose);
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

        List<Translation2d> ferryTargets = Util.isRedAlliance() ? kRedFerryTargets : kBlueFerryTargets;
        Translation2d ferryTarget = robotPose.getTranslation().nearest(ferryTargets);

        Translation2d shooterToTarget = ferryTarget.minus(getShooterPose(robotPose).getTranslation());
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
