// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/SubsystemBase.h>
#include <numbers>

#include <frc/AnalogGyro.h>
#include <AHRS.h>
#include <frc/geometry/Translation2d.h>
#include <frc/kinematics/SwerveDriveKinematics.h>
#include <frc/kinematics/SwerveDriveOdometry.h>
#include <units/angle.h>
#include <units/angular_acceleration.h>
#include <units/angular_velocity.h>
#include <frc/DoubleSolenoid.h>

#include "SweeveMods.h"


class Train : public frc2::SubsystemBase 
{
public:

  struct SwerveModules
  {
    SweeveMods frontleft;
    SweeveMods frontright;
    SweeveMods rearleft;
    SweeveMods rearright;
    frc::DoubleSolenoid &mPneumatic;
  };

  Train(SwerveModules &mSwerveModules);
  
   /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Swervey(units::meters_per_second_t xSpeed, units::meters_per_second_t ySpeed, units::radians_per_second_t rot,
             bool fieldRelative);
            
  void UPDATEODOMETRY();
  void Periodic() override;
  void SetSolenoid(frc::DoubleSolenoid::Value position); 

private:

  frc::Translation2d m_locationFrontLeft{+0_in, +0_in};
  frc::Translation2d m_locationFrontRight{+0_in, -0_in};
  frc::Translation2d m_locationRearLeft{-0_in, +0_in};
  frc::Translation2d m_locationRearRight{-0_in, -0_in};

  AHRS gyro{frc::SPI::kMXP};

  SwerveModules &mSwerveModules;

  // declared private and exposed only through public methods.
   frc::SwerveDriveKinematics<4> mKinematics{m_locationFrontLeft,
                                             m_locationFrontRight,
                                             m_locationRearLeft,
                                             m_locationRearRight};

  frc::SwerveDriveOdometry<4> mOdometry{mKinematics, gyro.GetRotation2d(), {mSwerveModules.frontleft.GetCurrentPosition(),
                                                                            mSwerveModules.frontright.GetCurrentPosition(),
                                                                            mSwerveModules.rearleft.GetCurrentPosition(),
                                                                            mSwerveModules.rearright.GetCurrentPosition()}};

};
