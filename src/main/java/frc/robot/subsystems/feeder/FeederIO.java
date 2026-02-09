package frc.robot.subsystems.feeder;

import org.littletonrobotics.junction.AutoLog;

public interface FeederIO {
    
    @AutoLog
    public static class FeederIOInputs {
        
        public double feederPosition = 0.0;
        public double feederVelocity = 0.0;
        public double feederVoltage = 0.0;
        public double feederCurrent = 0.0;
        public double feederTemperature = 0.0;
    }

    public default void updateInputs (FeederIOInputs inputs) {}
    public default void setFeederPercentage (double percentage) {}
    public default void setFeederVelocity (double position) {}
    public default void setFeederVoltage(double magnitude) {}
}
