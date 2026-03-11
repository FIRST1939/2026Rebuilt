package frc.robot.subsystems.shooter;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

public class EntryAngleShooterSolutionFinder implements ShooterSolutionFinder {

    private final LoggedNetworkNumber m_goalPosX =
            new LoggedNetworkNumber("/ShooterSolution/Goal Pos X", 8.23);
    private final LoggedNetworkNumber m_goalPosY =
            new LoggedNetworkNumber("/ShooterSolution/Goal Pos Y", 4.11);
    private final LoggedNetworkNumber m_hoodHeightM =
            new LoggedNetworkNumber("/ShooterSolution/Hood Height M",
                                    ShooterConstants.kDefaultHoodHeightMeters);

    private Solution m_latestSolution = new Solution(0, 0);

    @Override
    public Solution update(Pose2d robotPose, ChassisSpeeds robotSpeeds) {
        Translation2d goalPosition = new Translation2d(
                m_goalPosX.getAsDouble(), m_goalPosY.getAsDouble());

        double R = robotPose.getTranslation().getDistance(goalPosition);
        double h = ShooterConstants.kTargetHeightMeters - m_hoodHeightM.getAsDouble();

        m_latestSolution = calculate(R, h);
        return m_latestSolution;
    }

    @Override
    public Solution getLatestSolution() {
        return m_latestSolution;
    }

    Solution calculate(double R, double h) {
        double phi = Math.toRadians(ShooterConstants.kEntryAngleDegrees);

        // Validity check: h + R * tan(phi) must be positive for a real solution
        if (h + R * Math.tan(phi) <= 0) {
            logSolution(new Solution(0, 0), R, h, 0.0, 0.0, false);
            return new Solution(0, 0);
        }

        // From fixed-entry-angle projectile model:
        // Vx^2 = (g * R^2) / (2 * (h + R * tan(phi)))
        double g = 9.80665;
        double Vx2 = (g * R * R) / (2.0 * (h + R * Math.tan(phi)));
        double Vx = Math.sqrt(Vx2);

        // Vy = Vx * tan(phi) + g * R / Vx  ... derived from kinematics
        // Actually: at entry the ball must arrive at angle -phi (downward),
        // so Vy_entry = -Vx * tan(phi).
        // From kinematics: Vy_entry = Vy_launch - g * t, t = R / Vx
        // => Vy_launch = Vx * tan(phi) + g * (R / Vx)  ... wait, let me re-derive.
        //
        // Standard fixed-entry-angle result:
        //   launch angle θ satisfies: tan(θ) = (h/R + tan(φ)) / (1 - tan(φ) * h/R - tan²(φ))
        //   But the simpler closed-form is:
        //   Vx = sqrt(g*R^2 / (2*(h + R*tan(phi))))
        //   Vy (launch, upward positive) = Vx * tan(phi) + g*R/Vx ... NO
        //
        // Let t = time of flight = R / Vx.
        // Vertical: y(t) = Vy*t - 0.5*g*t^2 = h  => Vy = (h + 0.5*g*t^2) / t
        double t = R / Vx;
        double Vy = (h + 0.5 * g * t * t) / t;

        double V0 = Math.sqrt(Vx * Vx + Vy * Vy);
        double launchAngleDeg = Math.toDegrees(Math.atan2(Vy, Vx));

        Solution solution = new Solution(
                launchVelocityToRPM(V0),
                launchAngleToHoodRotations(launchAngleDeg));

        logSolution(solution, R, h, V0, launchAngleDeg, true);
        return solution;
    }

    private double launchVelocityToRPM(double mps) {
        return (mps * 60.0) / (2.0 * Math.PI * ShooterConstants.kFlywheelWheelRadiusMeters);
    }

    private double launchAngleToHoodRotations(double angleDeg) {
        return (90.0 - angleDeg) / 360.0;
    }

    // ---- Logging -------------------------------------------------------- //

    private void logSolution(Solution s, double range, double heightDiff,
                             double launchVelocityMps, double launchAngleDeg,
                             boolean physicsValid) {
        Logger.recordOutput("ShooterSolution/FlywheelRPM", s.flywheelRPM);
        Logger.recordOutput("ShooterSolution/HoodPositionRot", s.hoodPositionRotations);
        Logger.recordOutput("ShooterSolution/Range", range);
        Logger.recordOutput("ShooterSolution/HeightDiff", heightDiff);
        Logger.recordOutput("ShooterSolution/LaunchVelocityMps", launchVelocityMps);
        Logger.recordOutput("ShooterSolution/LaunchAngleDeg", launchAngleDeg);
        Logger.recordOutput("ShooterSolution/PhysicsValid", physicsValid);
    }
}
