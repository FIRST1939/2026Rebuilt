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
    // public ShooterParams getShotSolution(Pose2d robotPose)
    // {
    //     Translation2d shooterToTarget = Util.getHubPosition().minus(robotPose);
    //     ShooterParams shooterToTargetParams = kShooterMap.get(shooterToTargetDistance.getNorm());
    //     return shooterToTargetParams;
    // }
    
    public void calculateShotSolution(Pose2d robotPose, ChassisSpeeds robotSpeeds) {

        // Translation2d robotVelocity = new Translation2d(
        //     robotSpeeds.vxMetersPerSecond,
        //     robotSpeeds.vyMetersPerSecond);
        // Predict future robot pose

        double lookaheadTime = 0.1;
        //ShooterParams staticSolution getShotShotSolition(robotPose);

        Pose2d futureRobotPose = findFuturePose(robotPose, robotSpeeds, lookaheadTime + Constants.kTimeOfFlight);
        Logger.recordOutput("ShotSolver/FutureXPose", futureRobotPose.getX());
        Logger.recordOutput("ShotSolver/FutureYPose", futureRobotPose.getY());

        // Compute future shooter position
        Translation2d futureRobotPosition = futureRobotPose.getTranslation();

        Translation2d shooterPosition = futureRobotPose.transformBy(
            new Transform2d(
                ShooterConstants.kRobotToShooter, 
                new Rotation2d()
            )
        ).getTranslation();

        // Distance used for shooter map

        
        Translation2d robotCenterToTarget = Util.getHubPosition().minus(futureRobotPosition);
        
        double shooterToTargetDistance = robotCenterToTarget.getNorm();
        Logger.recordOutput("ShotSolver/distanceToTarget", shooterToTargetDistance);

        // Vector used for physics + aiming
        Translation2d shooterToTarget = Util.getHubPosition().minus(shooterPosition);

        // if (shooterToTargetDistance < 1e-3 || shooterToTarget.getNorm() < 1e-3) {
            
        //     m_shotSolution = new ShotSolution(0, 0, robotPose.getRotation());
        //     return;
        // }

        ShooterParams shooterToTargetParams = kShooterMap.get(shooterToTargetDistance);

        // double idealHorizontalSpeed = rawDistance / rawParams.kTimeOfFlight;

        // Translation2d idealShotDirection = shooterToTarget.div(shooterToTarget.getNorm());
        // Translation2d idealShotVector = idealShotDirection.times(idealHorizontalSpeed);
        // Translation2d compensatedShotVector = idealShotVector.minus(robotVelocity);

        // double compensatedSpeed = compensatedShotVector.getNorm();
        // double virtualDistance = rawDistance * (compensatedSpeed / idealHorizontalSpeed);

        // ShooterParams compensatedParams = kShooterMap.get(virtualDistance);

        Rotation2d aimHeading = shooterToTarget.getAngle().plus(new Rotation2d(Math.PI));
        Logger.recordOutput("ShotSolver/kFlyWheelRPM", shooterToTargetParams.kFlywheelRPM);
        Logger.recordOutput("ShotSolver/kHoodPositionRotations", shooterToTargetParams.kHoodPositionRotations);
        
        m_shotSolution = new ShotSolution(
            shooterToTargetParams.kFlywheelRPM,
            shooterToTargetParams.kHoodPositionRotations,
            aimHeading);
    }

    public ShotSolution getShotSolution () {

        return m_shotSolution;
    }
}

