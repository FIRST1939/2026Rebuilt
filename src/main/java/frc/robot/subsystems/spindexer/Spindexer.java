package frc.robot.subsystems.spindexer;

import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

public class Spindexer extends SubsystemBase {

    private final SpindexerIO m_io;
    private final SpindexerIOInputsAutoLogged m_inputs = new SpindexerIOInputsAutoLogged();
    private final SysIdRoutine sysIdRoutine;


    public Spindexer (SpindexerIO io) {

        m_io = io;

        this.sysIdRoutine = new SysIdRoutine(

            new SysIdRoutine.Config(
                Volts.per(Units.Second).of(SpindexerConstants.kSysIdRampUpTime), 
                Volts.of(SpindexerConstants.kSysIdVoltageIncrement), 
                Seconds.of(SpindexerConstants.kSysIdDuration)),

            new SysIdRoutine.Mechanism(
                voltage -> io.setSpindexerVoltage(voltage.magnitude()), 
                log -> {
                    log
                        .motor("spindexerMotor")
                        .voltage(Volts.of(this.m_inputs.spindexerVoltage))
                        .angularPosition(Rotations.of(this.m_inputs.spindexerPosition))
                        .angularVelocity(RotationsPerSecond.of(this.m_inputs.spindexerVelocity));
                },
                this, 
                "Spindexer")
        );
    }
 
    @Override
    public void periodic() {

        m_io.updateInputs(m_inputs);
        Logger.processInputs("Spindexer", m_inputs);

    }

    public void setSpindexerPercentage (double percentage) {
        this.m_io.setSpindexerPercentage(percentage);
    }

    public void setSpindexerVelocity (double velocity) {
        this.m_io.setSpindexerVelocity(velocity);
    }
    
    public double getSpindexerVelocity () {
        return this.m_inputs.spindexerVelocity;
    }

    public void setSpindexerVoltage(double magnitude) {
        this.m_io.setSpindexerVoltage(magnitude);
    }

    public Command SpindexerSysIdQuasistaticForward() {
        return sysIdRoutine.quasistatic(SysIdRoutine.Direction.kForward);
    }

    public Command SpindexerSysIdQuasistaticReverse() {
        return sysIdRoutine.quasistatic(SysIdRoutine.Direction.kReverse);
    }

    public Command SpindexerSysIdDynamicForward() {
        return sysIdRoutine.dynamic(SysIdRoutine.Direction.kForward);
    }

    public Command SpindexerSysIdDynamicReverse() {
        return sysIdRoutine.dynamic(SysIdRoutine.Direction.kReverse);
    }
}