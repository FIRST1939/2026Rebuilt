package frc.robot.subsystems.shooter;

import org.littletonrobotics.junction.AutoLog;

public interface ShooterIO {

    @AutoLog
    public static class ShooterIOInputs {
        
        public double shooterPosition = 0.0;
        public double shooterVelocity = 0.0;
        public double shooterVoltage = 0.0;
        public double shooterCurrent = 0.0;
        public double shooterTemperature = 0.0;
    }

    public default void updateInputs (ShooterIOInputs inputs) {}
}
