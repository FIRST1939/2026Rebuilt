package frc.robot.subsystems.spindexer;

import org.littletonrobotics.junction.AutoLog;

public interface SpindexerIO {
    
    @AutoLog
    public static class SpindexerIOInputs {
        
        public double spindexerPosition = 0.0;
        public double spindexerVelocity = 0.0;
        public double spindexerVoltage = 0.0;
        public double spindexerCurrent = 0.0;
        public double spindexerTemperature = 0.0;
    }

    public default void updateInputs (SpindexerIOInputs inputs) {}
    public default void setSpindexerPercentage (double percent) {}
    public default void setSpindexerVoltage(double magnitude) {}
    public default void setSpindexerVelocity (double velocity) {}
}
