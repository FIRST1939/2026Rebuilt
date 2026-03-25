package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Volts;
import static edu.wpi.first.units.Units.VoltsPerRadianPerSecond;
import static edu.wpi.first.units.Units.VoltsPerRadianPerSecondSquared;

import com.revrobotics.sim.SparkFlexSim;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;

public class ShooterIOSim extends ShooterIOHardware {

    private final SparkFlexSim m_flywheelLeaderSim = new SparkFlexSim(m_flywheelLeader, DCMotor.getNeoVortex(1));
    private final SparkFlexSim m_flywheelFollowerSim = new SparkFlexSim(m_flywheelFollower, DCMotor.getNeoVortex(1));
    private final SparkFlexSim m_hoodSim = new SparkFlexSim(m_hood, DCMotor.getNeoVortex(1));

    private final FlywheelSim m_flywheelPhysicsSim = new FlywheelSim(
        LinearSystemId.identifyVelocitySystem(
            Volts.of(ShooterConstants.kFlywheelFeedforwardV).per(RPM).in(VoltsPerRadianPerSecond),
            Volts.of(ShooterConstants.kFlywheelFeedforwardA).per(RPM.per(Second)).in(VoltsPerRadianPerSecondSquared)
        ),
        DCMotor.getNeoVortex(2)
    );

    private final FlywheelSim m_hoodPhysicsSim = new FlywheelSim(
        LinearSystemId.identifyVelocitySystem(
            Volts.of(ShooterConstants.kHoodFeedforwardV).per(RPM).in(VoltsPerRadianPerSecond),
            Volts.of(ShooterConstants.kHoodFeedforwardA).per(RPM.per(Second)).in(VoltsPerRadianPerSecondSquared)
        ),
        DCMotor.getNeoVortex(1)
    );

    @Override
    public void updateInputs(ShooterIOInputs inputs) {

        double flywheelVoltage = m_flywheelLeaderSim.getAppliedOutput() * m_flywheelLeaderSim.getBusVoltage();
        boolean flywheelOvercomeFriction = Math.abs(flywheelVoltage) > ShooterConstants.kFlywheelFeedforwardS;
        m_flywheelPhysicsSim.setInputVoltage(flywheelOvercomeFriction ? flywheelVoltage - Math.copySign(ShooterConstants.kFlywheelFeedforwardS, flywheelVoltage) : 0);
        m_flywheelPhysicsSim.update(0.02);

        m_flywheelLeaderSim.iterate(
            m_flywheelPhysicsSim.getAngularVelocityRPM(),
            RobotController.getBatteryVoltage(),
            0.02
        );

        m_flywheelFollowerSim.iterate(
            m_flywheelPhysicsSim.getAngularVelocityRPM(),
            RobotController.getBatteryVoltage(),
            0.02
        );

        double hoodVoltage = m_hoodSim.getAppliedOutput() * m_hoodSim.getBusVoltage() - ShooterConstants.kHoodFeedforwardG;
        boolean hoodOvercomeFriction = Math.abs(hoodVoltage) > ShooterConstants.kHoodFeedforwardS;
        m_hoodPhysicsSim.setInputVoltage(hoodOvercomeFriction ? hoodVoltage - Math.copySign(ShooterConstants.kHoodFeedforwardS, hoodVoltage) : 0);
        m_hoodPhysicsSim.update(0.02);

        m_hoodSim.iterate(
            m_hoodPhysicsSim.getAngularVelocityRPM(),
            RobotController.getBatteryVoltage(),
            0.02
        );

        super.updateInputs(inputs);
    }
}
