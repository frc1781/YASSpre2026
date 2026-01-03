package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Seconds;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import CRA.FeedForwardTuning;
import CRA.PIDTuning;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import yams.mechanisms.config.ElevatorConfig;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.ControlMode;
import yams.motorcontrollers.SmartMotorControllerConfig.MotorMode;
import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;

public class Elevator extends SubsystemBase {
    private SparkMax leftMotor;
    private SparkMax rightMotor;

    private SparkMaxConfig leftMotorConfig;
    private SparkMaxConfig rightMotorConfig;

    
    private ElevatorFeedforward elevatorFeedforward;
    private FeedForwardTuning elevatoFeedForwardTuning;
    private PIDController elevatorPID;
    private PIDTuning elevatorPIDTuning;
    private Shuffleboard tab;

    public Elevator() {
        leftMotor = new SparkMax(11, MotorType.kBrushless);
        rightMotor = new SparkMax(12, MotorType.kBrushless);

        leftMotorConfig = new SparkMaxConfig();
        leftMotorConfig.inverted(true);
        leftMotorConfig.smartCurrentLimit(38);
        leftMotorConfig.idleMode(IdleMode.kCoast);
        leftMotor.configure(leftMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        rightMotorConfig = new SparkMaxConfig();
        rightMotorConfig.follow(leftMotor, false);
        rightMotorConfig.smartCurrentLimit(38);
        rightMotorConfig.idleMode(IdleMode.kCoast);
        rightMotor.configure(rightMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    }

}
