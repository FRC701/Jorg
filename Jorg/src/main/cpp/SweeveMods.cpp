// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "SweeveMods.h"
#include <Constants.h>

SweeveMods::SweeveMods(SwerveBits &swerveBits)
    : mSwerveBits(swerveBits)
{
    ConfigureModules(Part::drive);
    ConfigureModules(Part::turn);
    ConfigureModules(Part::coder);
}

void SweeveMods::ConfigureModules(const Part &type)
{
    switch (type)
    {
    case Part::drive:
        mSwerveBits.mDriveMotor.SetInverted(ChassisConstants::DriveMotorDirection);
        break;

    case Part::turn:
        mSwerveBits.mTurnMotor.SetInverted(ChassisConstants::TurnMotorDirection);
        break;

    case Part::coder:
        mSwerveBits.mCanCoder.ConfigMagnetOffset(ChassisConstants::CanCoderOffset);
        break;
    }
}

double SweeveMods::GetAngle()
{
    return mSwerveBits.mCanCoder.GetAbsolutePosition();
}

double SweeveMods::GetSpeedmps()
{
    return (mSwerveBits.mDriveMotor.GetSelectedSensorVelocity() * GearRatios::DriveBox) / 60;
}