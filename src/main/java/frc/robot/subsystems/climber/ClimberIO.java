package frc.robot.subsystems.climber;

import org.littletonrobotics.junction.AutoLog;

public interface ClimberIO {
    
    @AutoLog
    public static class ClimberIOInputs {
        
        public double climberPosition = 0.0;
        public double climberVelocity = 0.0;
        public double climberVoltage = 0.0;
        public double climberCurrent = 0.0;
        public double climberTemperature = 0.0;
    }

    public default void updateInputs (ClimberIOInputs inputs) {}
}
