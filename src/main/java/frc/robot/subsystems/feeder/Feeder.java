package frc.robot.subsystems.feeder;

import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

public class Feeder extends SubsystemBase {
    
    private final FeederIO m_io;
    private final FeederIOInputsAutoLogged m_inputs = new FeederIOInputsAutoLogged();
    private final SysIdRoutine sysIdRoutine;

    public Feeder (FeederIO io) {
        
        m_io = io;

        this.sysIdRoutine = new SysIdRoutine(
            new SysIdRoutine.Config(
                Volts.per(Units.Second).of(FeederConstants.kSysIdRampUpTime), 
                Volts.of(FeederConstants.kSysIdVoltageIncrement), 
                Seconds.of(FeederConstants.kSysIdDuration)),

            new SysIdRoutine.Mechanism(
                voltage -> io.setFeederVoltage(voltage.magnitude()), 
                log -> {
                    log
                        .motor("intakeRoller")
                        .voltage(Volts.of(this.m_inputs.feederVoltage))
                        .angularPosition(Rotations.of(this.m_inputs.feederPosition))
                        .angularVelocity(RotationsPerSecond.of(this.m_inputs.feederVelocity));
                },
                this, 
                "Intake")
        );
    }

    @Override
    public void periodic () {

        m_io.updateInputs(m_inputs);
        Logger.processInputs("Feeder", m_inputs);
    }
    public void setFeederPercentage (double percent) {
      
        m_io.setFeederPercentage(percent);

    }
    
    public void setFeederVelocity (double velocity) {

        m_io.setFeederVelocity (velocity);
    }

    public double getFeederVelocity () {
        
        return this.m_inputs.feederVelocity;
    }

    public void setFeederVoltage(double magnitude) {
        this.m_io.setFeederVoltage(magnitude);
    }

    public Command FeederSysIdQuasistaticForward() {
        return sysIdRoutine.quasistatic(SysIdRoutine.Direction.kForward);
    }

    public Command FeederSysIdQuasistaticReverse() {
        return sysIdRoutine.quasistatic(SysIdRoutine.Direction.kReverse);
    }

    public Command FeederSysIdDynamicForward() {
        return sysIdRoutine.dynamic(SysIdRoutine.Direction.kForward);
    }

    public Command FeederSysIdDynamicReverse() {
        return sysIdRoutine.dynamic(SysIdRoutine.Direction.kReverse);
    }

}
