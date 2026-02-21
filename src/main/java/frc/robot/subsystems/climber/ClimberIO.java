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
    public default double getControllerSetpoint () { return 0; }
    public default void updateRaisingControllerFeedback (double kP, double kD) {}
    public default void updateClimbingControllerFeedback (double kP, double kD) {}
    public default void updateControllerProfile (double kCruiseVelocity, double kMaxAcceleration, double kAllowedError) {}
    public default void setClimberPercentage (double percent) {}
    public default void setClimberVoltage(double magnitude) {}
    public default void setRaisingPosition (double position) {}
    public default void setClimbingPosition (double position) {}
    public default boolean isAtSetpoint() {return false;}
}
