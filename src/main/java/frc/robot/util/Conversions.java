// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

import frc.robot.Constants.SwerveConstants;

/** Add your docs here. */
public class Conversions {
  public static double falconToDegrees(double falcon) {
    return falcon * (360 / (SwerveConstants.turnGearRatio * 2048));
  }

  public static double degreesToFalcon(double degrees) {
    return degrees * (SwerveConstants.turnGearRatio * 2048 / 360);
  }

  public static double falconToMeters(double falcon) {
    return falcon * SwerveConstants.drivePositionConversion / 2048;
  }

  public static double metersToFalcon(double meters) {
    return meters * 2048 / SwerveConstants.drivePositionConversion;
  }

  public static double falconToRPM(double falcon) {
    return falcon * (600 / 2048);
  }

  public static double rpmToFalcon(double rpm) {
    return rpm * (2048 / 600);
  }

  public static double falconToMPS(double falcon) {
    return falconToRPM(falcon) * SwerveConstants.driveVelocityConversion;
  }

  public static double mpsToFalcon(double mps) {
    double rpm = mps / SwerveConstants.driveVelocityConversion;

    return rpmToFalcon(rpm);
  }
}
