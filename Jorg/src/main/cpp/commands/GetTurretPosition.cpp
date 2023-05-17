// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include <frc/smartdashboard/SmartDashboard.h>

#include "commands/GetTurretPosition.h"

GetTurretPosition::GetTurretPosition(Turret& turret)
: mTurret(turret) 
{
  // Use addRequirements() here to declare subsystem dependencies.
}

// Called when the command is initially scheduled.
void GetTurretPosition::Initialize() {}

// Called repeatedly when this Command is scheduled to run
void GetTurretPosition::Execute() 
{
  double Pose = frc::SmartDashboard::GetNumber("turret angle", 0);
  mTurret.UseOutput(5, Pose);
}

// Called once the command ends or is interrupted.
void GetTurretPosition::End(bool interrupted) 
{
  mTurret.PowerSet(0);
}

// Returns true when the command should end.
bool GetTurretPosition::IsFinished() {
  return false;
}
