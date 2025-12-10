package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Nunchucks extends SubsystemBase {
    private final SparkMax turret = new SparkMax(0/*TEMP*/, MotorType.kBrushless);
    private final SparkMax base = new SparkMax(0/*TEMP*/, MotorType.kBrushless);
    private final SparkMax joint = new SparkMax(0/*TEMP*/, MotorType.kBrushless);
    private final SparkMaxConfig config = new SparkMaxConfig();
    
    private float turretPower = 0;
    private float basePower = 0;
    private float jointPower = 0;

    public Nunchucks() {
        config.idleMode(SparkBaseConfig.IdleMode.kBrake);
        config.smartCurrentLimit(30/*TEMP*/);
        turret.configure(config, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters);
        base.configure(config, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters);
        joint.configure(config, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters);

    }


    public void periodic() {
        turret.set(turretPower);
        base.set(basePower);
        joint.set(jointPower);
    }
}