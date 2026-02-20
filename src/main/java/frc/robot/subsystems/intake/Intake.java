package frc.robot.subsystems.intake;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

public class Intake extends SubsystemBase {

    private final IntakeIO m_io;
    private final IntakeIOInputsAutoLogged m_inputs = new IntakeIOInputsAutoLogged();
    private final SysIdRoutine m_rollerSysIdRoutine;
    private final SysIdRoutine m_leftPivotSysIdRoutine;
    private final SysIdRoutine m_rightPivotSysIdRoutine;

    public Intake (IntakeIO io) {

        m_io = io;

        m_rollerSysIdRoutine = new SysIdRoutine(
            new SysIdRoutine.Config(
                Volts.per(Units.Second).of(IntakeConstants.kRollerSysIdQuasistaticRampRate), 
                Volts.of(IntakeConstants.kRollerSysIdDynamicStepUp), 
                Seconds.of(IntakeConstants.kRollerSysIdDuration)),

            new SysIdRoutine.Mechanism(
                voltage -> m_io.setRollerVoltage(voltage.magnitude()), 
                log -> {
                    log
                        .motor("intakeRoller")
                        .voltage(Volts.of(m_inputs.rollerVoltage))
                        .angularPosition(Rotations.of(m_inputs.rollerPosition))
                        .angularVelocity(RotationsPerSecond.of(m_inputs.rollerVelocity));
                },
                this, 
                "Roller")
        );

        m_leftPivotSysIdRoutine = new SysIdRoutine(
            new SysIdRoutine.Config(
                Volts.per(Units.Second).of(IntakeConstants.kPivotSysIdQuasistaticRampRate), 
                Volts.of(IntakeConstants.kPivotSysIdDynamicStepUp), 
                Seconds.of(IntakeConstants.kPivotSysIdDuration)),

            new SysIdRoutine.Mechanism(
                voltage -> m_io.setLeftPivotVoltage(voltage.magnitude()), 
                log -> {
                    log
                        .motor("leftIntakePivot")
                        .voltage(Volts.of(m_inputs.leftPivotVoltage))
                        .angularPosition(Rotations.of(m_inputs.leftPivotPosition))
                        .angularVelocity(RotationsPerSecond.of(m_inputs.leftPivotVelocity));
                },
                this, 
                "Intake")
        );

        m_rightPivotSysIdRoutine = new SysIdRoutine(
            new SysIdRoutine.Config(
                Volts.per(Units.Second).of(IntakeConstants.kPivotSysIdQuasistaticRampRate), 
                Volts.of(IntakeConstants.kPivotSysIdDynamicStepUp), 
                Seconds.of(IntakeConstants.kPivotSysIdDuration)),

            new SysIdRoutine.Mechanism(
                voltage -> m_io.setRightPivotVoltage(voltage.magnitude()), 
                log -> {
                    log
                        .motor("rightIntakePivot")
                        .voltage(Volts.of(m_inputs.rightPivotVoltage))
                        .angularPosition(Rotations.of(m_inputs.rightPivotPosition))
                        .angularVelocity(RotationsPerSecond.of(m_inputs.rightPivotVelocity));
                },
                this, 
                "Intake")
        );
    }

    @Override
    public void periodic() {

        m_io.updateInputs(m_inputs);
        Logger.processInputs("Intake", m_inputs);
    }

    public void updateRollerControllerFeedback (double kP, double kD) {

        m_io.updateRollerControllerFeedback(kP, kD);
    }

    public double getAveragePivotControllerSetpoint () {

        return (m_io.getLeftPivotControllerSetpoint() + m_io.getRightPivotControllerSetpoint()) / 2.0;
    }

    public void updatePivotControllerFeedback (double kP, double kD) {

        m_io.updatePivotControllerFeedback(kP, kD);
    }

    public void updatePivotControllerProfile (double maxVelocity, double maxAcceleration, double allowedError) {

        m_io.updatePivotControllerProfile(maxVelocity, maxAcceleration, allowedError);
    }

    public double getRollerVelocity () {

        return m_inputs.rollerVelocity;
    }

    public double getRollerCurrent () {

        return m_inputs.rollerCurrent;
    }

    public double getAveragePivotPosition () {

        return (m_inputs.leftPivotPosition + m_inputs.rightPivotPosition) / 2.0;
    }

    public void setRollerPercentage (double percentage) {

        this.m_io.setRollerPercentage(percentage);
    }

    public void setRollerVelocity (double velocity) {

        this.m_io.setRollerVelocity(velocity);
    }

    public void setPivotPercentage (double percentage) {

        m_io.setLeftPivotPercentage(percentage);
        m_io.setRightPivotPercentage(percentage);
    }

    public void setPivotPosition (double position) {

        m_io.setLeftPivotPosition(position);
        m_io.setRightPivotPosition(position);
    }
    
    public Command rollerSysIdQuasistaticForward() {

        return m_rollerSysIdRoutine.quasistatic(SysIdRoutine.Direction.kForward);
    }

    public Command rollerSysIdQuasistaticReverse() {

        return m_rollerSysIdRoutine.quasistatic(SysIdRoutine.Direction.kReverse);
    }

    public Command rollerSysIdDynamicForward() {

        return m_rollerSysIdRoutine.dynamic(SysIdRoutine.Direction.kForward);
    }

    public Command rollerSysIdDynamicReverse() {

        return m_rollerSysIdRoutine.dynamic(SysIdRoutine.Direction.kReverse);
    }

    public Command leftPivotSysIdQuasistaticForward() {

        return m_leftPivotSysIdRoutine.quasistatic(SysIdRoutine.Direction.kForward);
    }

    public Command leftPivotSysIdQuasistaticReverse() {

        return m_leftPivotSysIdRoutine.quasistatic(SysIdRoutine.Direction.kReverse);
    }

    public Command leftPivotSysIdDynamicForward() {

        return m_leftPivotSysIdRoutine.dynamic(SysIdRoutine.Direction.kForward);
    }

    public Command leftPivotSysIdDynamicReverse() {

        return m_leftPivotSysIdRoutine.dynamic(SysIdRoutine.Direction.kReverse);
    }

    public Command rightPivotSysIdQuasistaticForward() {

        return m_rightPivotSysIdRoutine.quasistatic(SysIdRoutine.Direction.kForward);
    }

    public Command rightPivotSysIdQuasistaticReverse() {

        return m_rightPivotSysIdRoutine.quasistatic(SysIdRoutine.Direction.kReverse);
    }

    public Command rightPivotSysIdDynamicForward() {

        return m_rightPivotSysIdRoutine.dynamic(SysIdRoutine.Direction.kForward);
    }

    public Command rightPivotSysIdDynamicReverse() {

        return m_rightPivotSysIdRoutine.dynamic(SysIdRoutine.Direction.kReverse);
    }
}
