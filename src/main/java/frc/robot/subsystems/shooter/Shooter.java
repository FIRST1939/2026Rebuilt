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
    private final SysIdRoutine m_flywheelSysIdRoutine;
    private final SysIdRoutine m_hoodSysIdRoutine;
//Keep track of targets for isAtGoal.
    private double m_flywheelTargetVelocity = 0.0;
    private double m_hoodTargetPosition = 0.0;

    public Shooter (ShooterIO io) {

        m_io = io;

        m_flywheelSysIdRoutine = new SysIdRoutine(
            new SysIdRoutine.Config(
                Volts.per(Units.Second).of(ShooterConstants.kFlywheelSysIdQuasistaticRampRate), 
                Volts.of(ShooterConstants.kFlywheelSysIdDynamicStepUp), 
                Seconds.of(ShooterConstants.kFlywheelSysIdDuration)),

            new SysIdRoutine.Mechanism(
                voltage -> m_io.setFlywheelVoltage(voltage.magnitude()), 
                log -> {
                    log
                        .motor("flywheelMotor")
                        .voltage(Volts.of(m_inputs.flywheelLeaderVoltage))
                        .angularPosition(Rotations.of(getFlywheelPosition()))
                        .angularVelocity(RotationsPerSecond.of(getFlywheelVelocity()));
                },
                this, 
                "Flywheel")
        );

        m_hoodSysIdRoutine = new SysIdRoutine(
            new SysIdRoutine.Config(
                Volts.per(Units.Second).of(ShooterConstants.kHoodSysIdQuasistaticRampRate), 
                Volts.of(ShooterConstants.kHoodSysIdDynamicStepUp), 
                Seconds.of(ShooterConstants.kHoodSysIdDuration)),

            new SysIdRoutine.Mechanism(
                voltage -> m_io.setHoodVoltage(voltage.magnitude()), 
                log -> {
                    log
                        .motor("hoodMotor")
                        .voltage(Volts.of(m_inputs.hoodVoltage))
                        .angularPosition(Rotations.of(m_inputs.hoodPosition))
                        .angularVelocity(RotationsPerSecond.of(m_inputs.hoodVelocity));
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

    public void updateFlywheelControllerFeedback (double kP, double kD) {

        m_io.updateFlywheelControllerFeedback(kP, kD);
    }

    public void updateHoodControllerFeedback (double kP, double kD) {

        m_io.updateHoodControllerFeedback(kP, kD);
    }

    public double getFlywheelPosition () {

        return (m_inputs.flywheelLeaderPosition + m_inputs.flywheelFollowerPosition) / 2.0;
    }

    public double getFlywheelVelocity () {
        
        return (m_inputs.flywheelLeaderVelocity + m_inputs.flywheelFollowerVelocity) / 2.0;
    }

    public double getHoodPosition () {

        return m_inputs.hoodPosition;
    }

    public void setFlywheelPercentage (double percent) {

        m_io.setFlywheelPercentage(percent);
    }

    public void setFlywheelVelocity (double velocity) {

        m_flywheelTargetVelocity = velocity;
        m_io.setFlywheelVelocity(velocity);
    }

    /**
     * Returns true when the flywheel velocity is within the specified tolerance
     * of the target velocity set by {@link #setFlywheelVelocity(double)}.
     */
    public boolean isFlywheelAtGoal (double toleranceRPM) {

        return m_flywheelTargetVelocity != 0.0
            && Math.abs(getFlywheelVelocity() - m_flywheelTargetVelocity) < toleranceRPM;
    }


    public void setHoodPercentage (double percent) {

        m_io.setHoodPercentage(percent);
    }

    public void setHoodPosition (double position) {

        m_hoodTargetPosition = position;
        m_io.setHoodPosition(position);
    }

    /**
     * Returns true when the hood position is within the specified tolerance
     * of the target position set by {@link #setHoodPosition(double)}.
     */
    public boolean isHoodAtGoal (double toleranceRotations) {

        return Math.abs(getHoodPosition() - m_hoodTargetPosition) < toleranceRotations;
    }

    /**
     * Returns true when both the flywheel and hood are at their respective goals.
     */
    public boolean isAtGoal () {

        return isFlywheelAtGoal(ShooterConstants.kFlywheelToleranceRPM)
            && isHoodAtGoal(ShooterConstants.kHoodToleranceRotations);
    }

    public Command flywheelSysIdQuasistaticForward () {

        return m_flywheelSysIdRoutine.quasistatic(SysIdRoutine.Direction.kForward);
    }

    public Command flywheelSysIdQuasistaticReverse () {

        return m_flywheelSysIdRoutine.quasistatic(SysIdRoutine.Direction.kReverse);
    }

    public Command flywheelSysIdDynamicForward () {

        return m_flywheelSysIdRoutine.dynamic(SysIdRoutine.Direction.kForward);
    }

    public Command flywheelSysIdDynamicReverse () {

        return m_flywheelSysIdRoutine.dynamic(SysIdRoutine.Direction.kReverse);
    }
    
    public Command hoodSysIdQuasistaticForward() {

        return m_hoodSysIdRoutine.quasistatic(SysIdRoutine.Direction.kForward);
    }

    public Command hoodSysIdQuasistaticReverse() {

        return m_hoodSysIdRoutine.quasistatic(SysIdRoutine.Direction.kReverse);
    }

    public Command hoodSysIdDynamicForward() {

        return m_hoodSysIdRoutine.dynamic(SysIdRoutine.Direction.kForward);
    }

    public Command hoodSysIdDynamicReverse() {

        return m_hoodSysIdRoutine.dynamic(SysIdRoutine.Direction.kReverse);
    }
}
