package frc.robot.subsystems.intake;

public class IntakeConstants {

    public static final int kRollerCAN = 30;
    public static final double kRollerGearing = (20.0 / 40.0);
    public static final boolean kRollerInverted = false;

    public static final double kPivotGearing = (1.0 / 9.0) * (1.0 / 5.0);
    public static final int kPivotCurrentLimit = 40;

    public static final int kLeftPivotCAN = 31;
    public static final boolean kLeftPivotInverted = true;

    public static final int kRightPivotCAN = 32;
    public static final boolean kRightPivotInverted = false;

    public static final double kRollerSysIdQuasistaticRampRate = 0.5;
    public static final double kRollerSysIdDynamicStepUp = 6;
    public static final double kRollerSysIdDuration = 10.0;

    public static final double kRollerFeedforwardS = 0.13082;
    public static final double kRollerFeedforwardV = 0.0035432;
    public static final double kRollerFeedforwardA = 0.00017563;
    public static final double kRollerFeedbackP = 0.00025;

    public static final double kPivotSysIdQuasistaticRampRate = 0.1875;
    public static final double kPivotSysIdDynamicStepUp = 0.5625;
    public static final double kPivotSysIdDuration = 4.5;

    public static final double kLeftPivotFeedforwardS = 0.067857;
    public static final double kLeftPivotFeedforwardV = 0.026354;
    public static final double kLeftPivotFeedforwardA = 0.046195;
    public static final double kLeftPivotFeedforwardG = 0.0;

    public static final double kRightPivotFeedforwardS = 0.06291;
    public static final double kRightPivotFeedforwardV = 0.035861;
    public static final double kRightPivotFeedforwardA = 0.037106;
    public static final double kRightPivotFeedforwardG = 0.0;

    public static final double kLeftPivotFeedbackP = 7.5;
    public static final double kLeftPivotFeedbackD = 0.0;

    public static final double kRightPivotFeedbackP = 7.5;
    public static final double kRightPivotFeedbackD = 0.0;

    public static final double kPivotProfileCruiseVelocity = 0.0;
    public static final double kPivotProfileMaxAcceleration = 0.0;
    public static final double kPivotProfileAllowedError = 0.0;
}
