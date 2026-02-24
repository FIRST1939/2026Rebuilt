package frc.robot.subsystems.shooter;

import edu.wpi.first.math.geometry.Translation2d;

public class ShooterConstants {
        public static final double kFlywheelGearReduction = (1);

    
    public static final int kFlywheelLeaderCAN = 35;
    public static final double kFlywheelLeaderGearReduction = kFlywheelGearReduction;
    public static final boolean kFlywheelLeaderInverted = true;
    public static final int kFlywheelLeaderCurrentLimit = 120;

    public static final int kFlywheelFollowerCAN = 36;
    public static final double kFlywheelFollowerGearReduction = kFlywheelGearReduction;
    public static final boolean kFlywheelFollowerInverted = false;
    public static final int kFlywheelFollowerCurrentLimit = 120;

    public static final int kHoodCAN = 37;
    public static final double kHoodGearReduction = ((68.0 / 18.0) * (18.0 / 18.0) * (320.0 / 28.0));

    public static final int kFlywheelSysIdRampUpTime = 1;
    public static final int kFlywheelSysIdVoltageIncrement = 2;
    public static final int kFlywheelSysIdDuration = 10;

    public static final int kHoodSysIdRampUpTime = 1;
    public static final int kHoodSysIdVoltageIncrement = 2;
    public static final int kHoodSysIdDuration = 10;
    
    public static final int kHoodCurrentLimit = 120;

    // --- Regression / Field Geometry Constants ---

    /** Field-relative position of the scoring target (meters). Placeholder — replace before competition. */
    public static final Translation2d kTargetPosition = new Translation2d(0.0, 0.0);

    /** Height of the scoring target above the floor (meters). Placeholder. */
    public static final double kTargetHeightMeters = 2.0;

    /** Height of the shooter exit point above the floor (meters). Placeholder. */
    public static final double kShooterHeightMeters = 0.5;

    /**
     * Default desired entry angle of the ball at the target, in degrees.
     * Negative = downward trajectory at the target (standard convention).
     * Convert to radians when constructing EntryAngleShooterModel.
     */
    public static final double kDefaultEntryAngleDegrees = -45.0;
}
