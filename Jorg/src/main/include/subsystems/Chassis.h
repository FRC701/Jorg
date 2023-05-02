// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/SubsystemBase.h>
#include <ctre/Phoenix.h>
#include "Constants.h"
#include <frc/drive/DifferentialDrive.h>

using namespace ChassisConstants;

class Chassis : public frc2::SubsystemBase
{
public:
  struct ChassisList
  {
    WPI_TalonFX mFrontLeft;
    WPI_TalonFX mFrontRight;
    WPI_TalonFX mRearLeft;
    WPI_TalonFX mRearRight;
  };
  Chassis(ChassisList &chassisList);

  void ArcadeDrive(double speed, double rotation);

  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic() override;

private:
  // Components (e.g. motor controllers and sensors) should generally be
  // declared private and exposed only through public methods.
  ChassisList &mChassisList;
  frc::DifferentialDrive mDrive;
};
