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

public class IntakePivot extends SubsystemBase {

    private final IntakePivotIO m_io;
    private final IntakePivotIOInputsAutoLogged m_inputs = new IntakePivotIOInputsAutoLogged();
    private final SysIdRoutine m_leftPivotSysIdRoutine;
    private final SysIdRoutine m_rightPivotSysIdRoutine;

    public IntakePivot (IntakePivotIO io) {

        m_io = io;

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
        Logger.processInputs("IntakePivot", m_inputs);
    }

    public double getLeftPivotControllerPositionSetpoint () {

        return m_io.getLeftPivotControllerPositionSetpoint();
    }

    public double getLeftPivotControllerVelocitySetpoint () {

        return m_io.getLeftPivotControllerVelocitySetpoint();
    }

    public double getRightPivotControllerPositionSetpoint () {

        return m_io.getRightPivotControllerPositionSetpoint();
    }

    public double getRightPivotControllerVelocitySetpoint () {

        return m_io.getRightPivotControllerVelocitySetpoint();
    }

    public void updateLeftPivotControllerFeedback (double kP, double kD) {

        m_io.updateLeftPivotControllerFeedback(kP, kD);
    }

    public void updateRightPivotControllerFeedback (double kP, double kD) {

        m_io.updateRightPivotControllerFeedback(kP, kD);
    }

    public void updatePivotControllerProfile (double maxVelocity, double maxAcceleration, double allowedError) {

        m_io.updatePivotControllerProfile(maxVelocity, maxAcceleration, allowedError);
    }

    public double getLeftPivotPosition () {

        return m_inputs.leftPivotPosition;
    }

    public double getRightPivotPosition () {

        return m_inputs.rightPivotPosition;
    }

    public double getPivotPosition () {
        return ((m_inputs.rightPivotPosition + m_inputs.leftPivotPosition) / 2);
    }

    public void setPivotPercentage (double percentage) {

        m_io.setLeftPivotPercentage(percentage);
        m_io.setRightPivotPercentage(percentage);
    }

    public void setPivotPosition (double position) {

        m_io.setLeftPivotPosition(position);
        m_io.setRightPivotPosition(position);
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

    public boolean isPivotAtSetpoint() {
        return m_io.leftPivotIsAtSetpoint() && m_io.rightPivotIsAtSetpoint();
    }
}
