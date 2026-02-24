package frc.robot.subsystems.shooter;

public class ShooterConstants {
    
    public static final double kFlywheelGearing = 1.0;
    public static final boolean kFlywheelInverted = false;

    public static final int kFlywheelLeaderCAN = 35;
    public static final int kFlywheelFollowerCAN = 36;

    public static final int kHoodCAN = 37;
    public static final double kHoodGearing = (18.0 / 68.0) * (18.0 / 18.0) * (28.0 / 320.0);
    public static final int kHoodCurrentLimit = 20;
    public static final boolean kHoodInverted = true;

    public static final double kFlywheelSysIdQuasistaticRampRate = 0.5;
    public static final double kFlywheelSysIdDynamicStepUp = 4.0;
    public static final double kFlywheelSysIdDuration = 10.0;

    public static final double kFlywheelFeedforwardS = 0.21691;
    public static final double kFlywheelFeedforwardV = 0.0017369;
    public static final double kFlywheelFeedforwardA = 0.00034818;
    public static final double kFlywheelFeedbackP = 0.0004;

    public static final double kHoodSysIdQuasistaticRampRate = 0.05;
    public static final double kHoodSysIdDynamicStepUp = 0.2;
    public static final double kHoodSysIdDuration = 5.75;

    public static final double kHoodFeedforwardS = 0.096436;
    public static final double kHoodFeedforwardV = 0.06617;
    public static final double kHoodFeedforwardA = 0.0094421;
    public static final double kHoodFeedforwardG = 0.011553;

    public static final double kHoodFeedbackP = 250.0;
    public static final double kHoodFeedbackD = 0.0;

    // RPM tolerance for the flywheel to be considered "at goal". 
    public static final double kFlywheelToleranceRPM = 100.0;

    // Position tolerance (in rotations) for the hood to be considered "at goal".
    public static final double kHoodToleranceRotations = 1.0 / 360.0; // 1 degree
}
