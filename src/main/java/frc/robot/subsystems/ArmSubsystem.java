package frc.robot.subsystems;


import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.DegreesPerSecondPerSecond;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Pounds;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;
import static yams.mechanisms.SmartMechanism.gearbox;
import static yams.mechanisms.SmartMechanism.gearing;

import org.littletonrobotics.junction.Logger;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import yams.gearing.GearBox;
import yams.gearing.MechanismGearing;
import yams.mechanisms.config.ArmConfig;
import yams.mechanisms.config.MechanismPositionConfig;
import yams.mechanisms.positional.Arm;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.ControlMode;
import yams.motorcontrollers.SmartMotorControllerConfig.MotorMode;
import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;
import yams.motorcontrollers.local.SparkWrapper;

public class ArmSubsystem extends SubsystemBase
{

  private final SparkMax armMotor;

  private final SmartMotorControllerConfig motorConfig;
  private final SmartMotorController motor;
  private final MechanismPositionConfig robotToMechanism;

  private ArmConfig m_config;
  private final Arm arm;

  public ArmSubsystem ()
  {
    armMotor = new SparkMax(13, MotorType.kBrushless);

    motorConfig = new SmartMotorControllerConfig(this)
      .withClosedLoopController(4, 0, 0, DegreesPerSecond.of(180), DegreesPerSecondPerSecond.of(90))
      .withSoftLimit(Degrees.of(-75), Degrees.of(90))
      .withGearing(new MechanismGearing(GearBox.fromReductionStages(4, 5, 2 )))  //Disclaimer: Might not be completely right, needs to be checked later
      .withExternalEncoder(armMotor.getAbsoluteEncoder())
      .withZeroOffset(Rotations.of(0.315))
      .withIdleMode(MotorMode.BRAKE)
      .withTelemetry("ArmMotor", TelemetryVerbosity.HIGH)
      .withStatorCurrentLimit(Amps.of(30))
      .withVoltageCompensation(Volts.of(12))
      .withMotorInverted(false)
      .withClosedLoopRampRate(Seconds.of(0.25))
      .withOpenLoopRampRate(Seconds.of(0.25))
      .withFeedforward(new ArmFeedforward(0.0, 0.08, 0, 0))
      .withControlMode(ControlMode.CLOSED_LOOP);


    motor = new SparkWrapper(armMotor, DCMotor.getNEO(1), motorConfig);
    robotToMechanism = new MechanismPositionConfig()
      .withMaxRobotHeight(Meters.of(1))
      .withMaxRobotLength(Meters.of(1.2))
      .withRelativePosition(new Translation3d(Inches.of(5), Inches.of(0), Inches.of(20)));

    m_config = new ArmConfig(motor)
      .withLength(Inches.of(25))
      .withHardLimit(Degrees.of(-90), Degrees.of(90))
      .withTelemetry("Arm", TelemetryVerbosity.HIGH)
      .withMass(Pounds.of(1))
      .withStartingPosition(Degrees.of(80))
      .withHorizontalZero(Degrees.of(0))
      .withMechanismPositionConfig(robotToMechanism);
    arm = new Arm(m_config);
  }

  public void periodic()
  {
    Logger.recordOutput("Arm/position", arm.getAngle());
    arm.updateTelemetry();
  }

  public void simulationPeriodic()
  {
    arm.simIterate();
  }

  public Command armCmd(double dutycycle)
  {
    Logger.recordOutput("Arm/dc", dutycycle);
    return arm.set(dutycycle);
  }

  public Command sysId()
  {
    return arm.sysId(Volts.of(3), Volts.of(3).per(Second), Second.of(30));
  }

  public Command setAngle(Angle angle)
  {
    Logger.recordOutput("Arm/setAngle", angle);
    return arm.setAngle(angle);
  }

}