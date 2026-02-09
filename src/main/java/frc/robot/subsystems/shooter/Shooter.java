package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

public class Shooter extends SubsystemBase {

    private final ShooterIO m_io;
    private final ShooterIOInputsAutoLogged m_inputs = new ShooterIOInputsAutoLogged();
    private final SysIdRoutine flywheelSysIdRoutine;
    private final SysIdRoutine hoodSysIdRoutine;

    public Shooter (ShooterIO io) {
        m_io = io;
        this.flywheelSysIdRoutine = new SysIdRoutine(
            new SysIdRoutine.Config(
                Volts.per(Units.Second).of(ShooterConstants.kFlywheelSysIdRampUpTime), 
                Volts.of(ShooterConstants.kFlywheelSysIdVoltageIncrement), 
                Seconds.of(ShooterConstants.kFlywheelSysIdDuration)),

            new SysIdRoutine.Mechanism(
                voltage -> io.setFlywheelVoltage(voltage.magnitude()), 
                log -> {
                    log
                        .motor("flywheelMotor")
                        .voltage(Volts.of(this.m_inputs.flywheelLeaderVoltage))
                        .angularPosition(Rotations.of(getFlywheelPosition()))
                        .angularVelocity(RotationsPerSecond.of(this.m_inputs.flywheelLeaderVelocity));
                },
                this, 
                "Flywheel")
        );

        this.hoodSysIdRoutine = new SysIdRoutine(
            new SysIdRoutine.Config(
                Volts.per(Units.Second).of(ShooterConstants.kHoodSysIdRampUpTime), 
                Volts.of(ShooterConstants.kHoodSysIdVoltageIncrement), 
                Seconds.of(ShooterConstants.kHoodSysIdDuration)),

            new SysIdRoutine.Mechanism(
                voltage -> io.setHoodVoltage(voltage.magnitude()), 
                log -> {
                    log
                        .motor("hoodMotor")
                        .voltage(Volts.of(this.m_inputs.hoodVoltage))
                        .angularPosition(Rotations.of(getHoodPosition()))
                        .angularVelocity(RotationsPerSecond.of(this.m_inputs.hoodVelocity));
                },
                this, 
                "Hood")
        );
    }

    @Override    
    public void periodic () {
        m_io.updateInputs(m_inputs);
        Logger.processInputs("Shooter", m_inputs);
    }

    public void setFlywheelPercentage (double percent) {
        m_io.setFlywheelPercentage(percent);
    }

    public void setFlywheelVelocity (double percent) {
        m_io.setFlywheelVelocity(percent);
    }

    public double getFlywheelVelocity () {
        return this.m_inputs.flywheelLeaderVelocity;
    }


    public void setHoodPercentage (double percent) {
        m_io.setHoodPercentage(percent);
    }

    public void setHoodPosition (double percent) {
        m_io.setHoodPosition(percent);
    }

    public double getHoodPosition () {
        return this.m_inputs.hoodPosition;   
    }

    public double getFlywheelPosition () {
        return ((this.m_inputs.flywheelLeaderPosition + this.m_inputs.flywheelFollowerPosition) / 2);
    }

    public void setFlywheelVoltage(double magnitude) {
        this.m_io.setFlywheelPercentage(magnitude);
    }

    public void setHoodVoltage(double magnitude) {
        this.m_io.setHoodVoltage(magnitude);
    }

    public Command FlywheelSysIdQuasistaticForward() {
        return flywheelSysIdRoutine.quasistatic(SysIdRoutine.Direction.kForward);
    }

    public Command FlywheelSysIdQuasistaticReverse() {
        return flywheelSysIdRoutine.quasistatic(SysIdRoutine.Direction.kReverse);
    }

    public Command FlywheelSysIdDynamicForward() {
        return flywheelSysIdRoutine.dynamic(SysIdRoutine.Direction.kForward);
    }

    public Command FlywheelSysIdDynamicReverse() {
        return flywheelSysIdRoutine.dynamic(SysIdRoutine.Direction.kReverse);
    }
    
    public Command HoodSysIdQuasistaticForward() {
        return hoodSysIdRoutine.quasistatic(SysIdRoutine.Direction.kForward);
    }

    public Command HoodSysIdQuasistaticReverse() {
        return hoodSysIdRoutine.quasistatic(SysIdRoutine.Direction.kReverse);
    }

    public Command HoodSysIdDynamicForward() {
        return hoodSysIdRoutine.dynamic(SysIdRoutine.Direction.kForward);
    }

    public Command HoodSysIdDynamicReverse() {
        return hoodSysIdRoutine.dynamic(SysIdRoutine.Direction.kReverse);
    }
}
