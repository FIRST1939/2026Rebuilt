package frc.robot.subsystems.shooter;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;

public class ShooterIOHardware implements ShooterIO {
    
    protected final SparkFlex m_flywheelOne = new SparkFlex(ShooterConstants.kFlywheelOneCAN, MotorType.kBrushless);

    protected final SparkFlex m_flywheelTwo = new SparkFlex(ShooterConstants.kFlywheelTwoCAN, MotorType.kBrushless);

    protected final SparkFlex m_hood = new SparkFlex(ShooterConstants.kHoodCAN, MotorType.kBrushless);
}
