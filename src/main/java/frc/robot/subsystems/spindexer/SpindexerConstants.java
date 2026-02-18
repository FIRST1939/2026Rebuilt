package frc.robot.subsystems.spindexer;

public class SpindexerConstants {

    public static final int kSpindexerCAN = 33;
    public static final double kSpindexerGearing = (1.0/20.0)*(24.0/36.0);
    public static final boolean kInverted = true;

    public static final double kSysIdRampUpTime = 0.75;
    public static final double kSysIdVoltageIncrement = 7.5;
    public static final double kSysIdDuration = 10;

    public static final double kSpindexerFeedforwardS = 0.1811;
    public static final double kSpindexerFeedforwardV = 0.053177;
    public static final double kSpindexerFeedforwardA = 0.0027093;
}
