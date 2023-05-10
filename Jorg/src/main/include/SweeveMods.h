#include <frc/kinematics/SwerveDriveKinematics.h>
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <ctre/Phoenix.h>
#include <frc/geometry/Translation2d.h>
#include <frc/kinematics/SwerveDriveOdometry.h>
#include <frc2/command/SubsystemBase.h>
#include <frc/kinematics/SwerveModulePosition.h>
#include <units/length.h>
#include <frc/controller/ProfiledPIDController.h>
#include <frc/controller/PIDController.h>
#include <frc/controller/SimpleMotorFeedforward.h>
#include <frc/DoubleSolenoid.h>

class SweeveMods
{
public:
  struct SwerveBits
  {
    WPI_TalonFX mDriveMotor;
    WPI_TalonFX mTurnMotor;
    WPI_CANCoder mCanCoder;
  };

  SweeveMods(SwerveBits& swervebits);

  frc::SwerveModuleState GetCurrentState(); //const;
  frc::SwerveModulePosition GetCurrentPosition(); //const;

  void SetSwerveModuleState(const frc::SwerveModuleState &refstate);


  double GetSpeedmps();
  units::degree_t GetAngle();
private:

  frc::PIDController mdrivepid;
   frc::ProfiledPIDController<units::radians> mturnppid;
  frc::SimpleMotorFeedforward<units::radians> FFcontrol;
  SwerveBits &mSwerveBits;
};
