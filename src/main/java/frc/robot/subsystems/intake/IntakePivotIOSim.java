package frc.robot.subsystems.intake;

import com.revrobotics.sim.SparkFlexSim;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;

public class IntakePivotIOSim extends IntakePivotIOHardware {

    private static final double RPM_TO_RAD_S = 2.0 * Math.PI / 60.0;

    private final SparkFlexSim m_leftPivotMotorSim = new SparkFlexSim(m_leftPivotMotor, DCMotor.getNeoVortex(1));
    private final SparkFlexSim m_rightPivotMotorSim = new SparkFlexSim(m_rightPivotMotor, DCMotor.getNeoVortex(1));

    public static final double kPivotArmLengthMeters = 0.3;
    public static final double kPivotArmMassKg = 2.0;
    public static final double kPivotMinAngleRad = Math.toRadians(-5);
    public static final double kPivotMaxAngleRad = Math.toRadians(100);
    public static final double kPivotStartingAngleRad = Math.toRadians(0);

    private final SingleJointedArmSim m_leftPivotPhysicsSim = new SingleJointedArmSim(
        DCMotor.getNeoVortex(1),
        1.0 / IntakeConstants.kPivotGearing,
        SingleJointedArmSim.estimateMOI(kPivotArmLengthMeters, kPivotArmMassKg),
        kPivotArmLengthMeters,
        kPivotMinAngleRad,
        kPivotMaxAngleRad,
        true,
        kPivotStartingAngleRad
    );

    private final SingleJointedArmSim m_rightPivotPhysicsSim = new SingleJointedArmSim(
        DCMotor.getNeoVortex(1),
        1.0 / IntakeConstants.kPivotGearing,
        SingleJointedArmSim.estimateMOI(kPivotArmLengthMeters, kPivotArmMassKg),
        kPivotArmLengthMeters,
        kPivotMinAngleRad,
        kPivotMaxAngleRad,
        true,
        kPivotStartingAngleRad
    );

    @Override
    public void updateInputs(IntakePivotIOInputs inputs) {

        double leftPivotVoltage = m_leftPivotMotorSim.getAppliedOutput() * m_leftPivotMotorSim.getBusVoltage();
        m_leftPivotPhysicsSim.setInputVoltage(leftPivotVoltage);
        m_leftPivotPhysicsSim.update(0.02);

        m_leftPivotMotorSim.iterate(
            m_leftPivotPhysicsSim.getVelocityRadPerSec() / RPM_TO_RAD_S,
            RobotController.getBatteryVoltage(),
            0.02
        );

        double rightPivotVoltage = m_rightPivotMotorSim.getAppliedOutput() * m_rightPivotMotorSim.getBusVoltage();
        m_rightPivotPhysicsSim.setInputVoltage(rightPivotVoltage);
        m_rightPivotPhysicsSim.update(0.02);

        m_rightPivotMotorSim.iterate(
            m_rightPivotPhysicsSim.getVelocityRadPerSec() / RPM_TO_RAD_S,
            RobotController.getBatteryVoltage(),
            0.02
        );

        super.updateInputs(inputs);
    }
}
