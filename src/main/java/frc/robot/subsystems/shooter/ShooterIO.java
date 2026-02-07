package frc.robot.subsystems.shooter;

import org.littletonrobotics.junction.AutoLog;

public interface ShooterIO {

    @AutoLog
    public static class ShooterIOInputs {
        
        public double hoodPosition = 0.0;
        public double hoodVelocity = 0.0;
        public double hoodVoltage = 0.0;
        public double hoodCurrent = 0.0;
        public double hoodTemperature = 0.0;

        public double flywheelLeaderPosition = 0.0;
        public double flywheelLeaderVelocity = 0.0;
        public double flywheelLeaderVoltage = 0.0;
        public double flywheelLeaderCurrent = 0.0;
        public double flywheelLeaderTemperature = 0.0;

        public double flywheelFollowerVelocity = 0.0;
        public double flywheelFollowerPosition = 0.0;
        public double flywheelFollowerVoltage = 0.0;
        public double flywheelFollowerCurrent = 0.0;
        public double flywheelFollowerTemperature = 0.0;
    }

    public default void updateInputs (ShooterIOInputs inputs) {}
    public default void setFlywheelPercentage (double percent) {}
    public default void setFlywheelVelocity (double velocity) {}
    public default void setHoodPercentage (double percent) {}
    public default void setHoodPosition (double position) {}
    public default void setFlyWheelVoltage(double magnitude) {}
}
