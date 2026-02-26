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

public class IntakeRollers extends SubsystemBase {

    private final IntakeRollersIO m_io;
    private final IntakeRollersIOInputsAutoLogged m_inputs = new IntakeRollersIOInputsAutoLogged();
    private final SysIdRoutine m_rollerSysIdRoutine;

    public IntakeRollers (IntakeRollersIO io) {

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
    }

    @Override
    public void periodic() {

        m_io.updateInputs(m_inputs);
        Logger.processInputs("IntakeRollers", m_inputs);
    }

    public void updateRollerControllerFeedback (double kP, double kD) {

        m_io.updateRollerControllerFeedback(kP, kD);
    }

    public double getRollerVelocity () {

        return m_inputs.rollerVelocity;
    }

    public double getRollerCurrent () {

        return m_inputs.rollerCurrent;
    }

    public void setRollerPercentage (double percentage) {

        this.m_io.setRollerPercentage(percentage);
    }

    public void setRollerVelocity (double velocity) {

        this.m_io.setRollerVelocity(velocity);
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
}
