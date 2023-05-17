// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "RobotContainer.h"

#include <frc2/command/button/Trigger.h>
#include <frc/smartdashboard/SmartDashboard.h>

#include "commands/Autos.h"
#include "commands/ExampleCommand.h"
#include "commands/SwerveDrive.h"
#include "commands/GetTurretPosition.h"

RobotContainer::RobotContainer() {
  // Initialize all of your commands and subsystems here
  mTrain.SetDefaultCommand(SwerveDrive(mTrain, 
  [this] {return driver.GetX();},
  [this] {return driver.GetY();}, 
  [this] {return driver.GetTwist();
  frc::SmartDashboard::SetDefaultNumber("turret angle", 0);
  
  frc::SmartDashboard::PutData("turret Position", new GetTurretPosition(mTurret));
  }
  ));

  // Configure the button bindings
  ConfigureBindings();
}

void RobotContainer::ConfigureBindings() {
  // Configure your trigger bindings here

  // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
  frc2::Trigger([this] {
    return m_subsystem.ExampleCondition();
  }).OnTrue(ExampleCommand(&m_subsystem).ToPtr());

  // Schedule `ExampleMethodCommand` when the Xbox controller's B button is
  // pressed, cancelling on release.
  m_driverController.B().WhileTrue(m_subsystem.ExampleMethodCommand());
}

frc2::CommandPtr RobotContainer::GetAutonomousCommand() {
  // An example command will be run in autonomous
  return autos::ExampleAuto(&m_subsystem);
}
