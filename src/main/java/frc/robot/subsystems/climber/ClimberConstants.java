package frc.robot.subsystems.climber;

public class ClimberConstants {

    public static final int kClimberCAN = 38;
    public static final double kClimberGearing = (1.0 / 9.0);
    public static final int kCurrentLimit = 120;
    public static final boolean kInverted = true;

    public static final double kRaisingSysIdQuasistaticRampRate = 1.0;
    public static final double kRaisingSysIdDynamicStepUp = 2.0;
    public static final double kRaisingSysIdDuration = 10.0;

    public static final double kRaisingFeedforwardS = 0.1;
    public static final double kRaisingFeedforwardV = 0.1;
    public static final double kRaisingFeedforwardA = 0.1;
    public static final double kRaisingFeedforwardG = 0.1;

    public static final double kRaisingFeedbackP = 0.0;
    public static final double kRaisingFeedbackD = 0.0;

    public static final double kClimbingSysIdQuasistaticRampRate = 1.0;
    public static final double kClimbingSysIdDynamicStepUp = 2.0;
    public static final double kClimbingSysIdDuration = 10.0;

    public static final double kClimbingFeedforwardS = 0.1;
    public static final double kClimbingFeedforwardV = 0.1;
    public static final double kClimbingFeedforwardA = 0.1;
    public static final double kClimbingFeedforwardG = 0.1;

    public static final double kClimbingFeedbackP = 0.0;
    public static final double kClimbingFeedbackD = 0.0;

    public static final double kProfileCruiseVelocity = 0.0;
    public static final double kProfileMaxAcceleration = 0.0;
    public static final double kProfileAllowedError = 0.0;
}
