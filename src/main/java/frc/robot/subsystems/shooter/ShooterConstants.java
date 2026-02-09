package frc.robot.subsystems.shooter;

public class ShooterConstants {
        public static final double kFlywheelGearReduction = (1);

    
    public static final int kFlywheelLeaderCAN = 35;
    public static final double kFlywheelLeaderGearReduction = kFlywheelGearReduction;
    public static final boolean kFlywheelLeaderInverted = true;
    public static final int kFlywheelLeaderCurrentLimit = 120;

    public static final int kFlywheelFollowerCAN = 36;
    public static final double kFlywheelFollowerGearReduction = kFlywheelGearReduction;
    public static final boolean kFlywheelFollowerInverted = false;

    public static final int kHoodCAN = 37;
    public static final double kHoodGearReduction = ((68.0 / 18.0) * (18.0 / 18.0) * (320.0 / 28.0));

    public static final int kFlywheelSysIdRampUpTime = 1;
    public static final int kFlywheelSysIdVoltageIncrement = 2;
    public static final int kFlywheelSysIdDuration = 10;

    public static final int kHoodSysIdRampUpTime = 1;
    public static final int kHoodSysIdVoltageIncrement = 2;
    public static final int kHoodSysIdDuration = 10;
}
