package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.AutoLog;

public interface IntakeRollersIO {

    @AutoLog
    public static class IntakeRollersIOInputs {

        public double rollerPosition = 0.0;
        public double rollerVelocity = 0.0;
        public double rollerVoltage = 0.0;
        public double rollerCurrent = 0.0;
        public double rollerTemperature = 0.0;
    }

    public default void updateInputs (IntakeRollersIOInputs inputs) {}
    public default void updateRollerControllerFeedback (double kP, double kD) {}
    public default void setRollerPercentage (double percent) {}
    public default void setRollerVoltage(double voltage) {}
    public default void setRollerVelocity (double velocity) {}
}
