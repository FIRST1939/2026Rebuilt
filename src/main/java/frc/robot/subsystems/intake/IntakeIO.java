package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.AutoLog;

public interface IntakeIO {
    
    @AutoLog
    public static class IntakeIOInputs {
        
        public double intakePosition = 0.0;
        public double intakeVelocity = 0.0;
        public double intakeVoltage = 0.0;
        public double intakeCurrent = 0.0;
        public double intakeTemperature = 0.0;
    }

    public default void updateInputs (IntakeIOInputs inputs) {}
}
