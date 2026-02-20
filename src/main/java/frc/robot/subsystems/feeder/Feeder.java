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
    private final SysIdRoutine m_sysIdRoutine;

    public Feeder (FeederIO io) {
        
        m_io = io;

        m_sysIdRoutine = new SysIdRoutine(
            new SysIdRoutine.Config(
                Volts.per(Units.Second).of(FeederConstants.kSysIdQuasistaticRampRate), 
                Volts.of(FeederConstants.kSysIdDynamicStepUp), 
                Seconds.of(FeederConstants.kSysIdDuration)),

            new SysIdRoutine.Mechanism(
                voltage -> m_io.setFeederVoltage(voltage.magnitude()), 
                log -> {
                    log
                        .motor("feederMotor")
                        .voltage(Volts.of(m_inputs.feederVoltage))
                        .angularPosition(Rotations.of(m_inputs.feederPosition))
                        .angularVelocity(RotationsPerSecond.of(m_inputs.feederVelocity));
                },
                this, 
                "Feeder")
        );
    }

    @Override
    public void periodic () {

        m_io.updateInputs(m_inputs);
        Logger.processInputs("Feeder", m_inputs);
    }

    public void updateControllerFeedback (double kP, double kD) {

        m_io.updateControllerFeedback(kP, kD);
    }

    public double getFeederVelocity () {
        
        return this.m_inputs.feederVelocity;
    }

    public double getFeederCurrent () {

        return m_inputs.feederCurrent;
    }

    public void setFeederPercentage (double percent) {
      
        m_io.setFeederPercentage(percent);

    }
    
    public void setFeederVelocity (double velocity) {

        m_io.setFeederVelocity (velocity);
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
