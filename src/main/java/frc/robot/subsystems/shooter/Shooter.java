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
    private final SysIdRoutine sysIdRoutine;

    public Shooter (ShooterIO io) {
        m_io = io;
        this.sysIdRoutine = new SysIdRoutine(
            new SysIdRoutine.Config(
                Volts.per(Units.Second).of(ShooterConstants.kSysIdRampUpTime), 
                Volts.of(ShooterConstants.kSysIdVoltageIncrement), 
                Seconds.of(ShooterConstants.kSysIdDuration)),

            new SysIdRoutine.Mechanism(
                voltage -> io.setFlyWheelVoltage(voltage.magnitude()), 
                log -> {
                    log
                        .motor("intakeRoller")
                        .voltage(Volts.of(this.m_inputs.flywheelLeaderVoltage))
                        .angularPosition(Rotations.of(getFlywheelPosition()))
                        .angularVelocity(RotationsPerSecond.of(this.m_inputs.flywheelLeaderVelocity));
                },
                this, 
                "Intake")
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

     public Command sysIdQuasistaticForward() {
        return sysIdRoutine.quasistatic(SysIdRoutine.Direction.kForward);
    }

    public Command sysIdQuasistaticReverse() {
        return sysIdRoutine.quasistatic(SysIdRoutine.Direction.kReverse);
    }

    public Command sysIdDynamicForward() {
        return sysIdRoutine.dynamic(SysIdRoutine.Direction.kForward);
    }

    public Command sysIdDynamicReverse() {
        return sysIdRoutine.dynamic(SysIdRoutine.Direction.kReverse);
    }
}
