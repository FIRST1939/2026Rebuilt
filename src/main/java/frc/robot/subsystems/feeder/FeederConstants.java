package frc.robot.subsystems.feeder;

public class FeederConstants {
    
    public static final int kFeederCAN = 34;
    public static final double kFeederGearing = (18.0/54.0) * (24.0/24.0);

    public static final int kSysIdRampUpTime = 1;
    public static final int kSysIdVoltageIncrement = 2;
    public static final int kSysIdDuration = 10;

    public static final double kFeederFeedforwardS = 0.0;
    public static final double kFeederFeedforwardV = 0.0;
    public static final double kFeederFeedforwardA = 0.0;
}
