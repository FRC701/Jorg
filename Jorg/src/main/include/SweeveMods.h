// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <ctre/Phoenix.h>
#include <frc/geometry/Translation2d.h>
#include <frc/kinematics/SwerveDriveKinematics.h>
#include <frc/kinematics/SwerveDriveOdometry.h>
#include <frc2/command/SubsystemBase.h>
#include <frc/kinematics/SwerveModulePosition.h>
#include <units/length.h>

class SweeveMods
{
public:
  struct SwerveBits
  {
    WPI_TalonFX mDriveMotor;
    WPI_TalonFX mTurnMotor;
    WPI_CANCoder mCanCoder;
  };
  enum class Part{drive, turn, coder};
  void ConfigureModules(const Part& type);

  SweeveMods(SwerveBits& swervebits);

  frc::SwerveModuleState GetCurrentState() const;
  frc::SwerveModulePosition GetCurrentPosition() const;

  void SetSwerveModuleState(const frc::SwerveModuleState &state);

  double GetSpeedmps();
  double GetAngle();
private:

  SwerveBits &mSwerveBits;

};
