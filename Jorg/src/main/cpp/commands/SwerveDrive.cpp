// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/SwerveDrive.h"
#include <Constants.h>

SwerveDrive::SwerveDrive(Train &train, std::function<double()> xaxis,
                         std::function<double()> yaxis,
                         std::function<double()> rotation)
    : mTrain(train), Xaxis(xaxis), Yaxis(yaxis), Rotation(rotation)
{
  AddRequirements(&mTrain);
  // Use addRequirements() here to declare subsystem dependencies.
}

// Called when the command is initially scheduled.
void SwerveDrive::Initialize()
{
}

// Called repeatedly when this Command is scheduled to run
void SwerveDrive::Execute()
{
  mTrain.Swervey(XspeedLimiter.Calculate(frc::ApplyDeadband(Xaxis(), 0.02)) * units::meters_per_second_t(ChassisConstants::TOPSPEED),
                 YspeedLimiter.Calculate(frc::ApplyDeadband(Yaxis(), 0.02)) * units::meters_per_second_t(ChassisConstants::TOPSPEED),
                 RotLimiter.Calculate(frc::ApplyDeadband(Rotation() * 0, 0.20)) * ChassisConstants::MaxAngularVelocity,
                 false);
}

// Called once the command ends or is interrupted.
void SwerveDrive::End(bool interrupted) {}

// Returns true when the command should end.
bool SwerveDrive::IsFinished()
{
  return false;
}
