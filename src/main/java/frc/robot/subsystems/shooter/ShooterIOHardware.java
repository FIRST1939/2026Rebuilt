package frc.robot.subsystems.shooter;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;


public class ShooterIOHardware implements ShooterIO {
    
    protected final SparkFlex m_flywheelLeader = new SparkFlex(ShooterConstants.kFlywheelLeaderCAN, MotorType.kBrushless);
    protected final RelativeEncoder m_flywheelLeaderEncoder = m_flywheelLeader.getEncoder();
    protected final SparkClosedLoopController m_flywheelController = m_flywheelLeader.getClosedLoopController();

    protected final SparkFlex m_flywheelFollower = new SparkFlex(ShooterConstants.kFlywheelFollowerCAN, MotorType.kBrushless);
    protected final RelativeEncoder m_flywheelFollowerEncoder = m_flywheelFollower.getEncoder();

    protected final SparkFlex m_hood = new SparkFlex(ShooterConstants.kHoodCAN, MotorType.kBrushless);
    protected final RelativeEncoder m_hoodEncoder = m_hood.getEncoder();
    protected final SparkClosedLoopController m_hoodController = m_hood.getClosedLoopController();

    public ShooterIOHardware () {
       
        SparkFlexConfig globalFlywheelConfig = new SparkFlexConfig();

        globalFlywheelConfig
            .idleMode(IdleMode.kCoast)
            .voltageCompensation(12.0);

        globalFlywheelConfig.encoder
            .positionConversionFactor(ShooterConstants.kFlywheelGearing)
            .velocityConversionFactor(ShooterConstants.kFlywheelGearing);

        SparkFlexConfig flywheelLeaderConfig = new SparkFlexConfig();

        flywheelLeaderConfig
            .apply(globalFlywheelConfig)
            .inverted(ShooterConstants.kFlywheelLeaderInverted);
        
        m_flywheelLeader.configure(flywheelLeaderConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        SparkFlexConfig flywheelFollowerConfig = new SparkFlexConfig();

        flywheelFollowerConfig
            .apply(globalFlywheelConfig)
            .inverted(ShooterConstants.kFlywheelFollowerInverted)
            .follow(m_flywheelLeader);
        
        m_flywheelFollower.configure(flywheelFollowerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        SparkFlexConfig hoodConfig = new SparkFlexConfig();

        hoodConfig
            .idleMode(IdleMode.kBrake)
            .smartCurrentLimit(ShooterConstants.kHoodCurrentLimit)
            .voltageCompensation(12.0);

        hoodConfig.encoder
            .positionConversionFactor(1.0 / ShooterConstants.kHoodGearReduction)
            .velocityConversionFactor(1.0 / ShooterConstants.kHoodGearReduction);
        
        m_hood.configure(hoodConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    @Override
    public void updateInputs (ShooterIOInputs inputs) {

        inputs.flywheelLeaderPosition = m_flywheelLeaderEncoder.getPosition();
        inputs.flywheelLeaderVelocity = m_flywheelLeaderEncoder.getVelocity();
        inputs.flywheelLeaderVoltage = m_flywheelLeader.getAppliedOutput() * m_flywheelLeader.getBusVoltage();
        inputs.flywheelLeaderCurrent = m_flywheelLeader.getOutputCurrent();
        inputs.flywheelLeaderTemperature = m_flywheelLeader.getMotorTemperature();

        inputs.flywheelFollowerPosition = m_flywheelFollowerEncoder.getPosition();
        inputs.flywheelFollowerVelocity = m_flywheelFollowerEncoder.getVelocity();
        inputs.flywheelFollowerVoltage = m_flywheelFollower.getAppliedOutput() * m_flywheelFollower.getBusVoltage();
        inputs.flywheelFollowerCurrent = m_flywheelFollower.getOutputCurrent();
        inputs.flywheelFollowerTemperature = m_flywheelFollower.getMotorTemperature();

        inputs.hoodPosition = m_hoodEncoder.getPosition();
        inputs.hoodVelocity = m_hoodEncoder.getVelocity();
        inputs.hoodVoltage = m_hood.getAppliedOutput() * m_hood.getBusVoltage();
        inputs.hoodCurrent = m_hood.getOutputCurrent();
        inputs.hoodTemperature = m_hood.getMotorTemperature();
    }

    @Override
    public void setFlywheelPercentage (double percent) {

        m_flywheelLeader.set(percent);
    }

    @Override
    public void setFlywheelVoltage (double voltage) {

        m_flywheelController.setSetpoint(voltage, ControlType.kVoltage);
    }

    @Override
    public void setFlywheelVelocity (double velocity) {

        m_flywheelController.setSetpoint(velocity, ControlType.kMAXMotionVelocityControl);
    }

    @Override
    public void setHoodPercentage (double percent) {

        m_hood.set(percent);
    }   

    @Override
    public void setHoodVoltage (double voltage) {

        m_flywheelController.setSetpoint(voltage, ControlType.kVoltage);
    }   

    @Override
    public void setHoodPosition (double position) {

        m_flywheelController.setSetpoint(position, ControlType.kPosition);
    }    
}
