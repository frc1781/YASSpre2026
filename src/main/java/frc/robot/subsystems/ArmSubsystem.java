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

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import yams.gearing.GearBox;
import yams.gearing.MechanismGearing;
import yams.gearing.Sprocket;
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
 public GenericEntry armVoltageSet;
  private final SparkMax armMotor = new SparkMax(11, MotorType.kBrushless);
  private ShuffleboardTab tab;
    // private final SmartMotorControllerTelemetryConfig motorTelemetryConfig = new SmartMotorControllerTelemetryConfig()
    //       .withMechanismPosition()
    //      .withRotorPosition()
    //      .withMechanismLowerLimit()
    //       .withMechanismUpperLimit();

  private final SmartMotorControllerConfig motorConfig = new SmartMotorControllerConfig(this)
      .withClosedLoopController(0, 0, 0, DegreesPerSecond.of(180), DegreesPerSecondPerSecond.of(90))
      .withSoftLimit(Degrees.of(0), Degrees.of(80))
      .withGearing(new MechanismGearing(
        GearBox.fromReductionStages(5,5,4), Sprocket.fromStages("22:60")))
      //.withExternalEncoder(leftMotor.getAbsoluteEncoder())
     // .withZeroOffset(Rotations.of(0))
      .withIdleMode(MotorMode.COAST)
      .withTelemetry("armMotor", TelemetryVerbosity.HIGH)
//      .withSpecificTelemetry("armMotor", motorTelemetryConfig)
      .withStatorCurrentLimit(Amps.of(30))
      .withVoltageCompensation(Volts.of(12))
      .withMotorInverted(true)
      .withClosedLoopRampRate(Seconds.of(0.25))
      .withOpenLoopRampRate(Seconds.of(0.25))
      .withFeedforward(new ArmFeedforward(0.0, 0.08, 0, 0))
      .withControlMode(ControlMode.CLOSED_LOOP);


  private final SmartMotorController motor = new SparkWrapper(armMotor, DCMotor.getNEO(1), motorConfig);
  private final MechanismPositionConfig robotToMechanism = new MechanismPositionConfig()
      .withMaxRobotHeight(Meters.of(1.5))
      .withMaxRobotLength(Meters.of(0.75))
      .withRelativePosition(new Translation3d(Meters.of(0), Meters.of(0), Meters.of(0.5)));


  private ArmConfig m_config = new ArmConfig(motor)
      .withLength(Inches.of(39.5))
      .withHardLimit(Degrees.of(0), Degrees.of(80))
      .withTelemetry("Arm", TelemetryVerbosity.HIGH)
      .withMass(Pounds.of(1))
      .withStartingPosition(Degrees.of(0))
      //.withHorizontalZero(Degrees.of(0))
      .withMechanismPositionConfig(robotToMechanism);
  private final Arm arm = new Arm(m_config);
  

  public ArmSubsystem ()
  {
        SparkMaxConfig armMotorConfig = new SparkMaxConfig();
        armMotorConfig.idleMode(SparkMaxConfig.IdleMode.kCoast);

        tab = Shuffleboard.getTab(getName());
        armVoltageSet = tab.add(getName() + " armVoltageSet",  arm.getMotor().getVoltage().in(Volts)).getEntry();
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