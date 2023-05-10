// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/EngageGearshift.h"

EngageGearshift::EngageGearshift(Train& train) 
: mTrain(train)
{
  // Use addRequirements() here to declare subsystem dependencies.
}

// Called when the command is initially scheduled.
void EngageGearshift::Initialize() 
{
  mTrain.SetSolenoid(frc::DoubleSolenoid::kForward);
}

// Called repeatedly when this Command is scheduled to run
void EngageGearshift::Execute() 
{

}

// Called once the command ends or is interrupted.
void EngageGearshift::End(bool interrupted) 
{
  mTrain.SetSolenoid(frc::DoubleSolenoid::kReverse);
}

// Returns true when the command should end.
bool EngageGearshift::IsFinished() {
  return false;
}
