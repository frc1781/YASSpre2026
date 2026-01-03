package frc.robot.subsystems;

import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;

import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.util.function.FloatSupplier;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Nunchucks extends SubsystemBase {
    private final SparkMax turret = new SparkMax(20, MotorType.kBrushless);
    private final SparkMax base = new SparkMax(21, MotorType.kBrushless);
    private final SparkMax joint = new SparkMax(22, MotorType.kBrushless);
    private final SparkMaxConfig config = new SparkMaxConfig();
    
    private double turretPower = 0;
    private double basePower = 0;
    private double jointPower = 0;

    public Nunchucks() {
        config.idleMode(SparkBaseConfig.IdleMode.kBrake);
        config.smartCurrentLimit(30);
        turret.configure(config, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters);
        base.configure(config, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters);
        joint.configure(config, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters);
    }

    public void periodic() {
        Logger.recordOutput("Nunchucks/turretPower", turretPower);
        turret.set(turretPower);
        base.set(basePower);
        joint.set(jointPower);
    }

    public Command setTurretPower(DoubleSupplier value) {
        return new InstantCommand(() -> {
            turretPower = value.getAsDouble();
        }, this);
    }
}