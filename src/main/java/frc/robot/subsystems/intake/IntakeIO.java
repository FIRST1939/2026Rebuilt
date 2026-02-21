package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.AutoLog;

public interface IntakeIO {
    
    @AutoLog
    public static class IntakeIOInputs {
        
        public double rollerPosition = 0.0;
        public double rollerVelocity = 0.0;
        public double rollerVoltage = 0.0;
        public double rollerCurrent = 0.0;
        public double rollerTemperature = 0.0;

        public double leftPivotPosition = 0.0;
        public double leftPivotVelocity = 0.0;
        public double leftPivotVoltage = 0.0;
        public double leftPivotCurrent = 0.0;
        public double leftPivotTemperature = 0.0;

        public double rightPivotPosition = 0.0;
        public double rightPivotVelocity = 0.0;
        public double rightPivotVoltage = 0.0;
        public double rightPivotCurrent = 0.0;
        public double rightPivotTemperature = 0.0;
    }

    public default void updateInputs (IntakeIOInputs inputs) {}
    public default void updateRollerControllerFeedback (double kP, double kD) {}
    public default double getLeftPivotControllerSetpoint () { return 0; }
    public default double getRightPivotControllerSetpoint () { return 0; }
    public default void updateLeftPivotControllerFeedback(double kP, double kD) {}
    public default void updateRightPivotControllerFeedback(double kP, double kD) {}
    public default void updatePivotControllerProfile (double maxVelocity, double maxAcceleration, double allowedError) {}
    public default void setRollerPercentage (double percent) {}
    public default void setRollerVoltage(double voltage) {}
    public default void setRollerVelocity (double velocity) {}
    public default void setLeftPivotPercentage (double percent) {}
    public default void setLeftPivotVoltage(double voltage) {}
    public default void setLeftPivotPosition (double position) {}
    public default void setRightPivotPercentage (double percent) {}
    public default void setRightPivotVoltage(double voltage) {}
    public default void setRightPivotPosition (double position) {}
    public default boolean leftPivotIsAtSetpoint() {return false;}
    public default boolean rightPivotIsAtSetpoint() {return false;}
}
