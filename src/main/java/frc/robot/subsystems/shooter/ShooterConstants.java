package frc.robot.subsystems.shooter;

public class ShooterConstants {
    public static final int kFlywheelLeaderCAN = 35;
    public static final double kFlywheelLeaderGearReduction = (1);
    public static final boolean kFlywheelLeaderInverted = true;

    public static final int kFlywheelFollowerCAN = 36;
    public static final double kFlywheelFollowerGearReduction = (1);
    public static final boolean kFlywheelFollowerInverted = false;

    public static final int kHoodCAN = 37;
    public static final double kHoodGearReduction = ((68.0 / 18.0) * (18.0 / 18.0) * (320.0 / 28.0));
}
