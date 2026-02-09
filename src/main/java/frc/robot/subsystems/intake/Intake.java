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
    private final SysIdRoutine rollerSysIdRoutine;
    private final SysIdRoutine pivotSysIdRoutine;

    
    public Intake (IntakeIO io) {
        m_io = io;
        this.rollerSysIdRoutine = new SysIdRoutine(
            new SysIdRoutine.Config(
                Volts.per(Units.Second).of(IntakeConstants.kRollerSysIdRampUpTime), 
                Volts.of(IntakeConstants.kRollerSysIdVoltageIncrement), 
                Seconds.of(IntakeConstants.kRollerSysIdDuration)),

            new SysIdRoutine.Mechanism(
                voltage -> io.setRollerVoltage(voltage.magnitude()), 
                log -> {
                    log
                        .motor("intakeRoller")
                        .voltage(Volts.of(this.m_inputs.rollerVoltage))
                        .angularPosition(Rotations.of(this.m_inputs.rollerPosition))
                        .angularVelocity(RotationsPerSecond.of(this.m_inputs.rollerVelocity));
                },
                this, 
                "Roller")
        );

        this.pivotSysIdRoutine = new SysIdRoutine(
            new SysIdRoutine.Config(
                Volts.per(Units.Second).of(IntakeConstants.kPivotSysIdRampUpTime), 
                Volts.of(IntakeConstants.kPivotSysIdVoltageIncrement), 
                Seconds.of(IntakeConstants.kPivotSysIdDuration)),

            new SysIdRoutine.Mechanism(
                voltage -> io.setPivotVoltage(voltage.magnitude()), 
                log -> {
                    log
                        .motor("intakePivot")
                        .voltage(Volts.of(this.m_inputs.pivotLeaderVoltage))
                        .angularPosition(Rotations.of(getPivotPosition()))
                        .angularVelocity(RotationsPerSecond.of(this.m_inputs.pivotLeaderVelocity));
                },
                this, 
                "Pivot")
        );
    }

     @Override
    public void periodic() {

        m_io.updateInputs(m_inputs);
        Logger.processInputs("Intake", m_inputs);

    }

    public void setRollerPercentage (double percentage) {
        this.m_io.setRollerPercentage(percentage);
    }

    public void setRollerVelocity (double velocity) {
        this.m_io.setRollerVelocity(velocity);
    }
    
    public double getRollerVelocity () {
        return this.m_inputs.rollerVelocity;
    }

    public void setPivotPercentage (double percentage) {
        this.m_io.setPivotPercentage(percentage);
    }

    public void setPivotPosition (double position) {
        this.m_io.setPivotPosition(position);
    }
    
    public double getPivotPosition () {
        return ((this.m_inputs.pivotLeaderPosition + this.m_inputs.pivotFollowerPosition) / 2);
    }

    public void setRollerVoltage(double magnitude) {
        this.m_io.setRollerVoltage(magnitude);
    }

    public void setPivotVoltage(double magnitude) {
        this.m_io.setPivotVoltage(magnitude);
    }
    
    public Command RollerSysIdQuasistaticForward() {
        return rollerSysIdRoutine.quasistatic(SysIdRoutine.Direction.kForward);
    }

    public Command RollerSysIdQuasistaticReverse() {
        return rollerSysIdRoutine.quasistatic(SysIdRoutine.Direction.kReverse);
    }

    public Command RollerSysIdDynamicForward() {
        return rollerSysIdRoutine.dynamic(SysIdRoutine.Direction.kForward);
    }

    public Command RollerSysIdDynamicReverse() {
        return rollerSysIdRoutine.dynamic(SysIdRoutine.Direction.kReverse);
    }

    public Command PivotSysIdQuasistaticForward() {
        return pivotSysIdRoutine.quasistatic(SysIdRoutine.Direction.kForward);
    }

    public Command PivotSysIdQuasistaticReverse() {
        return pivotSysIdRoutine.quasistatic(SysIdRoutine.Direction.kReverse);
    }

    public Command PivotSysIdDynamicForward() {
        return pivotSysIdRoutine.dynamic(SysIdRoutine.Direction.kForward);
    }

    public Command PivotSysIdDynamicReverse() {
        return pivotSysIdRoutine.dynamic(SysIdRoutine.Direction.kReverse);
    }

}

