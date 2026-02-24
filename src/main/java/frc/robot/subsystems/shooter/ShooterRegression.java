package frc.robot.subsystems.shooter;

import edu.wpi.first.math.geometry.Translation2d;

/**
 * Abstract base class for all shooter ballistic models.
 * Implementations receive the robot floor-plane position and return
 * the required flywheel speed and hood angle.
 * Returns Double.NaN from both getters when the shot is physically invalid.
 */
public abstract class ShooterRegression {

    /** Launch speed in m/s, or Double.NaN if not achievable. */
    public abstract double getFlywheelVelocityMetersPerSecond(Translation2d robotPose, double targetHeightMeters);

    /** Hood angle in radians (from horizontal), or Double.NaN if not achievable. */
    public abstract double getHoodAngleRadians(Translation2d robotPose, double targetHeightMeters);

    /** True if the shot is physically achievable from this position. */
    public abstract boolean isValid(Translation2d robotPose, double targetHeightMeters);
}
