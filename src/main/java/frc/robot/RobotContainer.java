// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.*;
import yams.mechanisms.positional.Arm;

public class RobotContainer
{
  final CommandXboxController driverXbox = new CommandXboxController(0);
  final Nunchucks nunchucks = new Nunchucks();

  public RobotContainer()
  {
    configureBindings();

    DriverStation.silenceJoystickConnectionWarning(true);
  }

  private void configureBindings()
  {
    nunchucks.setDefaultCommand(nunchucks.setTurretPower(() -> {return 0;}));
    driverXbox.a().onTrue(new InstantCommand(() -> {System.out.println("press");}));
    driverXbox.a().whileTrue(nunchucks.setTurretPower(() -> {return 0.2;}));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand()
  {
    return null;
  }
}
