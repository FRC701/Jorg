// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/Chassis.h"

using namespace ChassisConstants;

Chassis::Chassis(ChassisList& chassisList)
: mChassisList(chassisList)
, mDrive(mChassisList.mRearLeft, mChassisList.mRearRight)
{
mChassisList.mFrontLeft.Follow(mChassisList.mRearLeft);
mChassisList.mFrontRight.Follow(mChassisList.mRearRight);
}

// This method will be called once per scheduler run
void Chassis::Periodic() {}

void Chassis::ArcadeDrive(double speed, double rotation) 
{
    mDrive.ArcadeDrive(speed, rotation);
}