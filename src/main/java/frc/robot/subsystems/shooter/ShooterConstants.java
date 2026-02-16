package frc.robot.subsystems.shooter;

public class ShooterConstants {
    
    public static final double kFlywheelGearing = 1.0;
    
    public static final int kFlywheelLeaderCAN = 35;
    public static final boolean kFlywheelLeaderInverted = true;

    public static final int kFlywheelFollowerCAN = 36;
    public static final boolean kFlywheelFollowerInverted = false;

    public static final int kHoodCAN = 37;
    public static final double kHoodGearing = (18.0 / 68.0) * (18.0 / 18.0) * (28.0 / 320.0);
    public static final int kHoodCurrentLimit = 5;

    public static final int kFlywheelSysIdRampUpTime = 1;
    public static final int kFlywheelSysIdVoltageIncrement = 2;
    public static final int kFlywheelSysIdDuration = 10;

    public static final double kFlywheelFeedforwardS = 0.1;
    public static final double kFlywheelFeedforwardV = 0.1;
    public static final double kFlywheelFeedforwardA = 0.1;

    public static final double kFlywheelFeedbackP = 0.0;
    public static final double kFlywheelFeedbackD = 0.0;

    public static final double kFlywheelProfileMaxAcceleration = 0.0;
    public static final double kFlywheelProfileAllowedError = 0.0;

    public static final int kHoodSysIdRampUpTime = 1;
    public static final int kHoodSysIdVoltageIncrement = 2;
    public static final int kHoodSysIdDuration = 10;

    public static final double kHoodFeedforwardS = 0.1;
    public static final double kHoodFeedforwardV = 0.1;
    public static final double kHoodFeedforwardA = 0.1;

    public static final double kHoodFeedbackP = 0.0;
    public static final double kHoodFeedbackD = 0.0;
}
