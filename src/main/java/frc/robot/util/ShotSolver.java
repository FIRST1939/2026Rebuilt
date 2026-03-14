package frc.robot.util;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;

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
        kShooterMap.put(1.93, new ShooterParams(2650, 0.0458, 1.14));
        kShooterMap.put(2.94, new ShooterParams(3000, 0.047, 1.31));
        kShooterMap.put(3.54, new ShooterParams(3250, 0.0458, 1.42));
        kShooterMap.put(5.70, new ShooterParams(3400, 0.07, 1.34));
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
    //     Translation2d shooterToTarget = Util.getHubPosition().minus(shooterPosition);
    //     ShooterParams shooterToTargetParams = kShooterMap.get(shooterToTargetDistance);
    //     return shooterToTargetParams;
    // }
    
    public void calculateShotSolution(Pose2d robotPose, ChassisSpeeds robotSpeeds) {

        // Translation2d robotVelocity = new Translation2d(
        //     robotSpeeds.vxMetersPerSecond,
        //     robotSpeeds.vyMetersPerSecond);
        // Predict future robot pose

        double lookaheadTime = 0.1;
        //ShooterParams staticSolution getShotShotSolition(robotPose);

        Pose2d futureRobotPose = findFuturePose(robotPose, robotSpeeds, lookaheadTime + Constants.kTimeOfFlight );

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

        // Vector used for physics + aiming
        Translation2d shooterToTarget = Util.getHubPosition().minus(shooterPosition);

        if (shooterToTargetDistance < 1e-3 || shooterToTarget.getNorm() < 1e-3) {
            
            m_shotSolution = new ShotSolution(0, 0, robotPose.getRotation());
            return;
        }

        ShooterParams shooterToTargetParams = kShooterMap.get(shooterToTargetDistance);

        // double idealHorizontalSpeed = rawDistance / rawParams.kTimeOfFlight;

        // Translation2d idealShotDirection = shooterToTarget.div(shooterToTarget.getNorm());
        // Translation2d idealShotVector = idealShotDirection.times(idealHorizontalSpeed);
        // Translation2d compensatedShotVector = idealShotVector.minus(robotVelocity);

        // double compensatedSpeed = compensatedShotVector.getNorm();
        // double virtualDistance = rawDistance * (compensatedSpeed / idealHorizontalSpeed);

        // ShooterParams compensatedParams = kShooterMap.get(virtualDistance);

        Rotation2d aimHeading = shooterToTarget.getAngle().plus(new Rotation2d(Math.PI));

        m_shotSolution = new ShotSolution(
            shooterToTargetParams.kFlywheelRPM,
            shooterToTargetParams.kHoodPositionRotations,
            aimHeading);
    }

    public ShotSolution getShotSolution () {

        return m_shotSolution;
    }
}

