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

        public double pivotLeaderPosition = 0.0;
        public double pivotLeaderVelocity = 0.0;
        public double pivotLeaderVoltage = 0.0;
        public double pivotLeaderCurrent = 0.0;
        public double pivotLeaderTemperature = 0.0;

        public double pivotFollowerPosition = 0.0;
        public double pivotFollowerVelocity = 0.0;
        public double pivotFollowerVoltage = 0.0;
        public double pivotFollowerCurrent = 0.0;
        public double pivotFollowerTemperature = 0.0;
    }

    public default void updateInputs (IntakeIOInputs inputs) {}
    public default void setRollerPercentage (double percent) {}
    public default void setRollerVoltage(double voltage) {}
    public default void setRollerVelocity (double velocity) {}
    public default void setPivotPercentage (double percent) {}
    public default void setPivotVoltage(double voltage) {}
    public default void setPivotPosition (double position) {}
}
