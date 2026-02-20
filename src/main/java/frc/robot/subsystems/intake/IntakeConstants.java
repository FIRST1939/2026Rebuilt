package frc.robot.subsystems.intake;

public class IntakeConstants {

    public static final int kRollerCAN = 30;
    public static final double kRollerGearing = (20.0 / 40.0);
    public static final boolean kRollerInverted = false;

    public static final double kPivotGearing = (1.0 / 5.0) * (1.0 / 5.0);
    public static final int kPivotCurrentLimit = 120;

    public static final int kLeftPivotCAN = 31;
    public static final boolean kLeftPivotInverted = true;

    public static final int kRightPivotCAN = 32;
    public static final boolean kRightPivotInverted = false;

    public static final double kRollerSysIdRampUpTime = 0.5;
    public static final double kRollerSysIdVoltageIncrement = 6;
    public static final int kRollerSysIdDuration = 10;

    public static final double kRollerFeedforwardS = 0.13082;
    public static final double kRollerFeedforwardV = 0.0035432;
    public static final double kRollerFeedforwardA = 0.00017563;

    public static final double kRollerFeedbackP = 0.00025;
    public static final double kRollerFeedbackD = 0.0;

    public static final int kPivotSysIdRampUpTime = 1;
    public static final int kPivotSysIdVoltageIncrement = 2;
    public static final int kPivotSysIdDuration = 10;

    public static final double kPivotFeedforwardS = 0.1;
    public static final double kPivotFeedforwardV = 0.1;
    public static final double kPivotFeedforwardA = 0.1;
    public static final double kPivotFeedforwardG = 0.1;

    public static final double kPivotFeedbackP = 0.0;
    public static final double kPivotFeedbackD = 0.0;

    public static final double kPivotProfileCruiseVelocity = 0.0;
    public static final double kPivotProfileMaxAcceleration = 0.0;
    public static final double kPivotProfileAllowedError = 0.0;
}
