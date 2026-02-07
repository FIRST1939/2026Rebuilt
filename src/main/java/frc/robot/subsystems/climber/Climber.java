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
    private final SysIdRoutine sysIdRoutine;

    public Climber (ClimberIO io) {

        m_io = io;

        this.sysIdRoutine = new SysIdRoutine(
            new SysIdRoutine.Config(
                Volts.per(Units.Second).of(ClimberConstants.kSysIdRampUpTime), 
                Volts.of(ClimberConstants.kSysIdVoltageIncrement), 
                Seconds.of(ClimberConstants.kSysIdDuration)),

            new SysIdRoutine.Mechanism(
                voltage -> io.setClimberVoltage(voltage.magnitude()), 
                log -> {
                    log
                        .motor("intakeRoller")
                        .voltage(Volts.of(this.m_inputs.climberVoltage))
                        .angularPosition(Rotations.of(this.m_inputs.climberPosition))
                        .angularVelocity(RotationsPerSecond.of(this.m_inputs.climberVelocity));
                },
                this, 
                "Intake")
        );
    }

    @Override
    public void periodic () {

        m_io.updateInputs(m_inputs);
        Logger.processInputs("Climber", m_inputs);
    }

    public void setClimberPercentage (double percent) {
        
        m_io.setClimberPercentage(percent);
    }
    
    public void setClimberPosition(double position) {
        m_io.setClimberPosition(position);
    }

    public double getClimberPosition() {
        return this.m_inputs.climberPosition;
    }

    public void setClimberVoltage(double magnitude) {
        m_io.setClimberVoltage(magnitude);
    }

    public Command ClimberSysIdQuasistaticForward() {
        return sysIdRoutine.quasistatic(SysIdRoutine.Direction.kForward);
    }

    public Command ClimberSysIdQuasistaticReverse() {
        return sysIdRoutine.quasistatic(SysIdRoutine.Direction.kReverse);
    }

    public Command ClimberSysIdDynamicForward() {
        return sysIdRoutine.dynamic(SysIdRoutine.Direction.kForward);
    }

    public Command ClimberSysIdDynamicReverse() {
        return sysIdRoutine.dynamic(SysIdRoutine.Direction.kReverse);
    }
}
