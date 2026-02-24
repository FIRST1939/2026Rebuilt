package frc.robot.subsystems.shooter;

import com.revrobotics.sim.SparkFlexSim;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;

public class ShooterIOSim extends ShooterIOHardware {

    private static final double RPM_TO_RAD_S = 2.0 * Math.PI / 60.0;

    private final SparkFlexSim m_flywheelLeaderSim = new SparkFlexSim(m_flywheelLeader, DCMotor.getNeoVortex(1));
    private final SparkFlexSim m_flywheelFollowerSim = new SparkFlexSim(m_flywheelFollower, DCMotor.getNeoVortex(1));
    private final SparkFlexSim m_hoodSim = new SparkFlexSim(m_hood, DCMotor.getNeoVortex(1));

    private final FlywheelSim m_flywheelPhysicsSim = new FlywheelSim(
        LinearSystemId.identifyVelocitySystem(
            ShooterConstants.kFlywheelFeedforwardV / RPM_TO_RAD_S,
            ShooterConstants.kFlywheelFeedforwardA / RPM_TO_RAD_S
        ),
        DCMotor.getNeoVortex(1)
    );

    private final FlywheelSim m_hoodPhysicsSim = new FlywheelSim(
        LinearSystemId.identifyVelocitySystem(
            ShooterConstants.kHoodFeedforwardV / RPM_TO_RAD_S,
            ShooterConstants.kHoodFeedforwardA / RPM_TO_RAD_S
        ),
        DCMotor.getNeoVortex(1)
    );

    @Override
    public void updateInputs(ShooterIOInputs inputs) {

        double flywheelVoltage = m_flywheelLeaderSim.getAppliedOutput() * m_flywheelLeaderSim.getBusVoltage();
        m_flywheelPhysicsSim.setInputVoltage(flywheelVoltage - Math.copySign(ShooterConstants.kFlywheelFeedforwardS, flywheelVoltage));
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

        double hoodVoltage = m_hoodSim.getAppliedOutput() * m_hoodSim.getBusVoltage();
        m_hoodPhysicsSim.setInputVoltage(hoodVoltage - Math.copySign(ShooterConstants.kHoodFeedforwardS, hoodVoltage));
        m_hoodPhysicsSim.update(0.02);

        m_hoodSim.iterate(
            m_hoodPhysicsSim.getAngularVelocityRPM(),
            RobotController.getBatteryVoltage(),
            0.02
        );

        super.updateInputs(inputs);
    }
}
