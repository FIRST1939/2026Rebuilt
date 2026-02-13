package frc.robot.subsystems.spindexer;

public class SpindexerConstants {

    public static final int kSpindexerCAN = 33;
    public static final double kSpindexerGearing = (1.0/20.0)*(24.0/36.0);
    public static int kCurrentLimit = 120;
    public static final boolean kInverted = false;

    public static final int kSysIdRampUpTime = 1;
    public static final int kSysIdVoltageIncrement = 12;
    public static final int kSysIdDuration = 10;

    public static final double kSpindexerFeedforwardS = 0.09129;
    public static final double kSpindexerFeedforwardV = 0.053608;
    public static final double kSpindexerFeedforwardA = 0.00027996;

}
