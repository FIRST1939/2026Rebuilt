package frc.robot.subsystems.intake;

public class IntakeConstants {

    public static final int kRollerCAN = 30;
    public static final double kRollerGearing = (20.0 / 40.0);
    public static final boolean kRollerInverted = false;

    public static final double kPivotGearing = (1.0 / 5.0) * (1.0 / 5.0);
    public static final int kPivotCurrentLimit = 120;

    public static final int kLeftPivotCAN = 31;
    public static final boolean kLeftPivotInverted = false;

    public static final int kRightPivotCAN = 32;
    public static final boolean kRightPivotInverted = false;

    public static final int kRollerSysIdRampUpTime = 1;
    public static final double kRollerSysIdVoltageIncrement = 0.1;
    public static final int kRollerSysIdDuration = 10;

    public static final double kRollerFeedforwardS = 0.1;
    public static final double kRollerFeedforwardV = 0.1;
    public static final double kRollerFeedforwardA = 0.1;

    public static final int kPivotSysIdRampUpTime = 1;
    public static final int kPivotSysIdVoltageIncrement = 2;
    public static final int kPivotSysIdDuration = 10;
}
