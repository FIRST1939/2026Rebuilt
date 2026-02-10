package frc.robot.subsystems.intake;

public class IntakeConstants {

    public static final int kRollerCAN = 30;
    public static final double kRollerGearing = (20.0/40.0);

    public static final int kPivotLeaderCAN = 31;
    public static final double kPivotLeaderGearReduction = ((1.0/45.0));

    public static final int kPivotFollowerCAN = 32;
    public static final double kPivotFollowerGearReduction = ((1.0/45.0));

    public static final int kRollerCurrentLimit = 120;
    public static final boolean kInverted = false;

    public static final int kRollerSysIdRampUpTime = 1;
    public static final int kRollerSysIdVoltageIncrement = 2;
    public static final int kRollerSysIdDuration = 10;

    public static final int kPivotSysIdRampUpTime = 1;
    public static final int kPivotSysIdVoltageIncrement = 2;
    public static final int kPivotSysIdDuration = 10;
}
