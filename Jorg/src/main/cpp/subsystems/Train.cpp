// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/Train.h"
#include <Constants.h>
#include <frc/smartdashboard/SmartDashboard.h>

 Train::Train(SwerveModules &mSwerveModules)
 : mSwerveModules(mSwerveModules)
 {
    gyro.Reset();
    mSwerveModules.frontleft.ConfigureModules(SweeveMods::Part::coder, ChassisConstants::CanCoderOffsetfl);
    mSwerveModules.frontright.ConfigureModules(SweeveMods::Part::coder, ChassisConstants::CanCoderOffsetfr);
    mSwerveModules.rearleft.ConfigureModules(SweeveMods::Part::coder, ChassisConstants::CanCoderOffsetrl);
    mSwerveModules.rearright.ConfigureModules(SweeveMods::Part::coder, ChassisConstants::CanCoderOffsetrr);
 }
    // This method will be called once per scheduler run
    void Train::Swervey(units::meters_per_second_t xSpeed,
             units::meters_per_second_t ySpeed,
             units::radians_per_second_t rot, bool fieldRelative)
    {
     auto modularStates = mKinematics.ToSwerveModuleStates(
        fieldRelative ? frc::ChassisSpeeds::FromFieldRelativeSpeeds(
                            xSpeed, ySpeed, rot, gyro.GetRotation2d())
                      : frc::ChassisSpeeds{xSpeed, ySpeed, rot});

   mKinematics.DesaturateWheelSpeeds(&modularStates, units::meters_per_second_t(ChassisConstants::TOPSPEED));
    
    auto [fl, fr, rl, rr] = modularStates;

    mSwerveModules.frontleft.SetSwerveModuleState(fl);  
    mSwerveModules.frontright.SetSwerveModuleState(fr);
    mSwerveModules.rearleft.SetSwerveModuleState(rl);
    mSwerveModules.rearright.SetSwerveModuleState(rr);
    }

    void Train::UPDATEODOMETRY(){
        mOdometry.Update(gyro.GetRotation2d(), {mSwerveModules.frontleft.GetCurrentPosition(),
                                               mSwerveModules.frontright.GetCurrentPosition(),
                                               mSwerveModules.rearleft.GetCurrentPosition(),
                                               mSwerveModules.rearright.GetCurrentPosition()});
    }

    void Train::Periodic() {
    UPDATEODOMETRY();

    frc::SmartDashboard::PutNumber("Front Left Angle", mSwerveModules.frontleft.GetAngle());
    frc::SmartDashboard::PutNumber("Front Right Angle",  mSwerveModules.frontright.GetAngle());
    frc::SmartDashboard::PutNumber("Rear Left Angle", mSwerveModules.rearleft.GetAngle());
    frc::SmartDashboard::PutNumber("Rear Right Angle",  mSwerveModules.rearright.GetAngle());

    frc::SmartDashboard::PutNumber("Front Left Speed MPS", mSwerveModules.frontleft.GetSpeedmps());
    frc::SmartDashboard::PutNumber("Front Right Speed MPS",  mSwerveModules.frontright.GetSpeedmps());
    frc::SmartDashboard::PutNumber("Rear Left Speed MPS", mSwerveModules.rearleft.GetSpeedmps());
    frc::SmartDashboard::PutNumber("Rear Right Speed MPS",  mSwerveModules.rearright.GetSpeedmps());
    }
