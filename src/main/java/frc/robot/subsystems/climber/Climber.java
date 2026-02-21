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
    private final SysIdRoutine m_raisingSysIdRoutine;
    private final SysIdRoutine m_climbingSysIdRoutine;

    public Climber (ClimberIO io) {

        m_io = io;

        m_raisingSysIdRoutine = new SysIdRoutine(
            new SysIdRoutine.Config(
                Volts.per(Units.Second).of(ClimberConstants.kRaisingSysIdQuasistaticRampRate), 
                Volts.of(ClimberConstants.kRaisingSysIdDynamicStepUp), 
                Seconds.of(ClimberConstants.kRaisingSysIdDuration)),

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

        m_climbingSysIdRoutine = new SysIdRoutine(
            new SysIdRoutine.Config(
                Volts.per(Units.Second).of(ClimberConstants.kClimbingSysIdQuasistaticRampRate), 
                Volts.of(ClimberConstants.kClimbingSysIdDynamicStepUp), 
                Seconds.of(ClimberConstants.kClimbingSysIdDuration)),

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

    public double getControllerSetpoint () {

        return m_io.getControllerSetpoint();
    }

    public void updateRaisingControllerFeedback (double kP, double kD) {

        m_io.updateRaisingControllerFeedback(kP, kD);
    }

    public void updateClimbingControllerFeedback (double kP, double kD) {

        m_io.updateClimbingControllerFeedback(kP, kD);
    }

    public void updateControllerProfile (double cruiseVelocity, double maxAcceleration, double allowedError) {

        m_io.updateControllerProfile(cruiseVelocity, maxAcceleration, allowedError);
    }

    public double getClimberPosition () {

        return m_inputs.climberPosition;
    }

    public void setClimberPercentage (double percent) {
        
        m_io.setClimberPercentage(percent);
    }

    public void setRaisingPosition (double position) {

        m_io.setRaisingPosition(position);
    }
    
    public void setClimbingPosition (double position) {

        m_io.setClimbingPosition(position);
    }

    public Command raisingSysIdQuasistaticForward () {

        return m_raisingSysIdRoutine.quasistatic(SysIdRoutine.Direction.kForward);
    }

    public Command raisingSysIdQuasistaticReverse () {

        return m_raisingSysIdRoutine.quasistatic(SysIdRoutine.Direction.kReverse);
    }

    public Command raisingSysIdDynamicForward () {

        return m_raisingSysIdRoutine.dynamic(SysIdRoutine.Direction.kForward);
    }

    public Command raisingSysIdDynamicReverse () {

        return m_raisingSysIdRoutine.dynamic(SysIdRoutine.Direction.kReverse);
    }

    public Command climbingSysIdQuasistaticForward () {

        return m_climbingSysIdRoutine.quasistatic(SysIdRoutine.Direction.kForward);
    }

    public Command climbingSysIdQuasistaticReverse () {

        return m_climbingSysIdRoutine.quasistatic(SysIdRoutine.Direction.kReverse);
    }

    public Command climbingSysIdDynamicForward () {

        return m_climbingSysIdRoutine.dynamic(SysIdRoutine.Direction.kForward);
    }

    public Command climbingSysIdDynamicReverse () {

        return m_climbingSysIdRoutine.dynamic(SysIdRoutine.Direction.kReverse);
    }

    public boolean atSetpoint() {
        return m_io.isAtSetpoint();
    }
}
