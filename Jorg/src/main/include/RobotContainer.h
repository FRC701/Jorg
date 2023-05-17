// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/CommandPtr.h>
#include <frc2/command/button/CommandXboxController.h>
#include <SweeveMods.h>

#include "Constants.h"
#include "subsystems/ExampleSubsystem.h"
#include "subsystems/Chassis.h"
#include "subsystems/Train.h"
#include "frc2/command/button/CommandJoystick.h"
#include "subsystems/Turret.h"

/**
 * This class is where the bulk of the robot should be declared.  Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls).  Instead, the structure of the robot (including subsystems,
 * commands, and trigger mappings) should be declared here.
 */

  using namespace TurretConstants;

class RobotContainer {
 public:
  RobotContainer();

  frc2::CommandPtr GetAutonomousCommand();

 private:
  // Replace with CommandPS4Controller or CommandJoystick if needed
  
   frc2::CommandJoystick driver{0};

  frc2::CommandXboxController m_driverController{
      OperatorConstants::kDriverControllerPort};

  // The robot's subsystems are defined here...
  ExampleSubsystem m_subsystem;
  
  Turret::TurretSystems mTurretElectronics{kTurretMotor, kTurretGyro};
  Turret mTurret{mTurretElectronics};

  SweeveMods::SwerveBits mSwerveModule1{kFrontLeftMod, kFrontLeftTurnMod, kFrontLeftCoder};
  SweeveMods::SwerveBits mSwerveModule2{kFrontRightMod, kFrontRightTurnMod, kFrontRightCoder};
  SweeveMods::SwerveBits mSwerveModule3{kRearLeftMod, kRearLeftTurnMod, kRearLeftCoder};
  SweeveMods::SwerveBits mSwerveModule4{kRearRightMod, kRearRightTurnMod, kRearRightCoder};

  Train::SwerveModules mSwerveModules{mSwerveModule1, mSwerveModule2, mSwerveModule3, mSwerveModule4};
  Train mTrain{mSwerveModules};  

  Chassis::ChassisList mChassisList{kFrontLeft, kFrontRight, kRearLeft, kRearRight};
  Chassis mChassis{mChassisList};
  void ConfigureBindings();
};
