package frc.robot.subsystems.feeder;

public class FeederConstants {
    
    public static final int kFeederCAN = 34;
    public static final boolean kInverted = false;
    public static final double kFeederGearing = (18.0 / 54.0) * (24.0 / 24.0);

    public static final double kSysIdRampUpTime = 0.625;
    public static final int kSysIdVoltageIncrement = 6;
    public static final int kSysIdDuration = 10;

    public static final double kFeederFeedforwardS = 0.11571;
    public static final double kFeederFeedforwardV = 0.0052932;
    public static final double kFeederFeedforwardA = 0.00010337;

    public static final double kFeederFeedbackP = 0.00025;
    public static final double kFeederFeedbackD = 0.0;
}
