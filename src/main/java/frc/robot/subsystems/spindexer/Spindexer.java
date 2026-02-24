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
    private final SysIdRoutine m_sysIdRoutine;

    public Spindexer (SpindexerIO io) {

        m_io = io;

        m_sysIdRoutine = new SysIdRoutine(

            new SysIdRoutine.Config(
                Volts.per(Units.Second).of(SpindexerConstants.kSysIdQuasistaticRampRate), 
                Volts.of(SpindexerConstants.kSysIdDynamicStepUp), 
                Seconds.of(SpindexerConstants.kSysIdDuration)),

            new SysIdRoutine.Mechanism(
                voltage -> m_io.setSpindexerVoltage(voltage.magnitude()), 
                log -> {
                    log
                        .motor("spindexerMotor")
                        .voltage(Volts.of(m_inputs.spindexerVoltage))
                        .angularPosition(Rotations.of(m_inputs.spindexerPosition))
                        .angularVelocity(RotationsPerSecond.of(m_inputs.spindexerVelocity));
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

    public double getSpindexerCurrent() {

        return m_inputs.spindexerCurrent;
    }

    public void setSpindexerPercentage (double percentage) {

        m_io.setSpindexerPercentage(percentage);
    }

    public void setSpindexerVelocity (double velocity) {

        m_io.setSpindexerVelocity(velocity);
    }

    public Command sysIdQuasistaticForward () {

        return m_sysIdRoutine.quasistatic(SysIdRoutine.Direction.kForward);
    }

    public Command sysIdQuasistaticReverse () {

        return m_sysIdRoutine.quasistatic(SysIdRoutine.Direction.kReverse);
    }

    public Command sysIdDynamicForward () {

        return m_sysIdRoutine.dynamic(SysIdRoutine.Direction.kForward);
    }

    public Command sysIdDynamicReverse () {

        return m_sysIdRoutine.dynamic(SysIdRoutine.Direction.kReverse);
    }
}
