package frc.robot.subsystems.climber;

import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

public class Climber extends SubsystemBase {

    private final ClimberIO m_io;
    private final ClimberIOInputsAutoLogged m_inputs = new ClimberIOInputsAutoLogged();
    private final SysIdRoutine m_sysIdRoutine;

    public Climber (ClimberIO io) {

        m_io = io;

        m_sysIdRoutine = new SysIdRoutine(
            new SysIdRoutine.Config(
                Volts.per(Units.Second).of(ClimberConstants.kSysIdRampUpTime), 
                Volts.of(ClimberConstants.kSysIdVoltageIncrement), 
                Seconds.of(ClimberConstants.kSysIdDuration)),

            new SysIdRoutine.Mechanism(
                voltage -> m_io.setClimberVoltage(voltage.magnitude()), 
                log -> {
                    log
                        .motor("climberMotor")
                        .voltage(Volts.of(m_inputs.climberVoltage))
                        .angularPosition(Rotations.of(m_inputs.climberPosition))
                        .angularVelocity(RotationsPerSecond.of(m_inputs.climberVelocity));
                },
                this, 
                "Climber")
        );
    }

    @Override
    public void periodic () {

        m_io.updateInputs(m_inputs);
        Logger.processInputs("Climber", m_inputs);
    }

    public double getClimberPosition () {

        return m_inputs.climberPosition;
    }

    public void setClimberPercentage (double percent) {
        
        m_io.setClimberPercentage(percent);
    }
    
    public void setClimberPosition (double position) {

        m_io.setClimberPosition(position);
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
