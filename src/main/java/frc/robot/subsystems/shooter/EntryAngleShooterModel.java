package frc.robot.subsystems.shooter;

import edu.wpi.first.math.geometry.Translation2d;

/**
 * Shooter ballistic model that constrains the ball's entry angle at the target
 * to a fixed, configurable value.
 *
 * <p>Physics derivation (projectile motion, entry-angle constrained):
 * <pre>
 *   R   = floor-plane distance from robot to target
 *   h   = targetHeightMeters - kShooterHeightMeters  (vertical displacement, caller-supplied)
 *   φ   = desired entry angle at target (radians; negative = downward)
 *   g   = 9.81 m/s²
 *
 *   Validity: h + R·tan(φ) > 0  AND  R > 1e-6
 *
 *   V_x = sqrt(g·R / (2·(h + R·tan(φ))))
 *   V_y = (g·R / V_x)  −  V_x·tan(φ)
 *   θ   = atan2(V_y, V_x)    ← hood angle (from horizontal)
 *   V_0 = sqrt(V_x² + V_y²)  ← launch speed
 * </pre>
 */
public class EntryAngleShooterModel extends ShooterRegression {

    private static final double kG = 9.81; // m/s²

    private final double m_entryAngleRadians;

    /**
     * @param entryAngleRadians Desired entry angle at the target in radians.
     *                          Use a negative value for a downward trajectory
     *                          (e.g., {@code Math.toRadians(-45)}).
     */
    public EntryAngleShooterModel(double entryAngleRadians) {
        m_entryAngleRadians = entryAngleRadians;
    }

    /** @return The configured entry angle in radians (for logging/diagnostics). */
    public double getEntryAngleRadians() {
        return m_entryAngleRadians;
    }

    @Override
    public boolean isValid(Translation2d robotPose, double targetHeightMeters) {
        double R = getHorizontalDistance(robotPose);
        if (R <= 1e-6) return false;
        double h = targetHeightMeters - ShooterConstants.kShooterHeightMeters;
        return (h + R * Math.tan(m_entryAngleRadians)) > 0;
    }

    @Override
    public double getFlywheelVelocityMetersPerSecond(Translation2d robotPose, double targetHeightMeters) {
        if (!isValid(robotPose, targetHeightMeters)) return Double.NaN;
        double R = getHorizontalDistance(robotPose);
        double h = targetHeightMeters - ShooterConstants.kShooterHeightMeters;
        double tanPhi = Math.tan(m_entryAngleRadians);
        double Vx = Math.sqrt(kG * R / (2.0 * (h + R * tanPhi)));
        double Vy = (kG * R / Vx) - Vx * tanPhi;
        return Math.sqrt(Vx * Vx + Vy * Vy);
    }

    @Override
    public double getHoodAngleRadians(Translation2d robotPose, double targetHeightMeters) {
        if (!isValid(robotPose, targetHeightMeters)) return Double.NaN;
        double R = getHorizontalDistance(robotPose);
        double h = targetHeightMeters - ShooterConstants.kShooterHeightMeters;
        double tanPhi = Math.tan(m_entryAngleRadians);
        double Vx = Math.sqrt(kG * R / (2.0 * (h + R * tanPhi)));
        double Vy = (kG * R / Vx) - Vx * tanPhi;
        return Math.atan2(Vy, Vx);
    }

    private double getHorizontalDistance(Translation2d robotPose) {
        return robotPose.getDistance(ShooterConstants.kTargetPosition);
    }
}
