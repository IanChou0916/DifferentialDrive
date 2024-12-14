// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static frc.robot.Constants.DRIVE_CONTROLLER_PORT;


import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.subsystem.DriveTrain;

public class RobotContainer {
  private final DriveTrain m_driveTrain = new DriveTrain();
  private final XboxController driveController = new XboxController(DRIVE_CONTROLLER_PORT);  
  private SendableChooser <Command> autoChooser = new SendableChooser<>();
  public RobotContainer() {
    setAutoCommand();
    SmartDashboard.putData("Auto Mode", autoChooser);
    configureBindings();
  }

  private void configureBindings() {
    m_driveTrain.setDefaultCommand(
      m_driveTrain.DriveCommand(driveController::getLeftY, driveController::getRightX));
    new JoystickButton(driveController, XboxController.Button.kA.value)
      .onTrue(m_driveTrain.sysIdDynamic(Direction.kForward));
      new JoystickButton(driveController, XboxController.Button.kB.value)
      .onTrue(m_driveTrain.sysIdDynamic(Direction.kReverse));
      new JoystickButton(driveController, XboxController.Button.kX.value)
      .onTrue(m_driveTrain.sysIdQuasistatic(Direction.kForward));
      new JoystickButton(driveController, XboxController.Button.kY.value)
      .onTrue(m_driveTrain.sysIdQuasistatic(Direction.kReverse));

    
  }

  public Command getAutonomousCommand() {

    return autoChooser.getSelected();
  }

  private void setAutoCommand() {
    autoChooser.addOption("sysIdDynamicForward", m_driveTrain.sysIdDynamic(Direction.kForward));
    autoChooser.addOption("sysIdDynamicReverse", m_driveTrain.sysIdDynamic(Direction.kReverse));
    autoChooser.addOption("sysIDQuasistaticForward", m_driveTrain.sysIdQuasistatic(Direction.kForward));
    autoChooser.addOption("sysIDQuasistaticReverse", m_driveTrain.sysIdQuasistatic(Direction.kReverse));
  }
}
