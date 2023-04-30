// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class OperatorConstants {
    public static final int driverControllerPort = 0;
  }

  public static class SwerveConstants {
    public static final double WIDTH = 0.353;
    public static final double LENGTH = 0.353;

    public static final Translation2d frontLeft = new Translation2d(WIDTH * 0.5, LENGTH * 0.5);
    public static final Translation2d frontRight = new Translation2d(WIDTH * 0.5, LENGTH * -0.5);
    public static final Translation2d backLeft = new Translation2d(WIDTH * -0.5, LENGTH * 0.5);
    public static final Translation2d backRight = new Translation2d(WIDTH * -0.5, LENGTH * -0.5);

    public static final Translation2d[] wheelLocations = { frontLeft, frontRight, backLeft, backRight };

    public static final double maxTranslationalSpeed = 4.0;
    public static final double maxAngularSpeed = Units.degreesToRadians(180.0);

    public static final double wheelCircumference = Units.inchesToMeters(4) * Math.PI;
    public static final double gearRatio = 1 / 8.41;

    public static final double drivePositionConversion = wheelCircumference * gearRatio;
    public static final double driveVelocityConversion = drivePositionConversion / 60;

    public static final double driveP = 0.25;

    public static final double driveKs = 0.2;
    public static final double driveKv = 2.5;
    public static final double driveKa = 0.34;

    public static final double turnP = 3.25;
    public static final double turnD = 0.10;

    public static final double maxModuleSpeed = 6.0;

    public static class AutoConstants {
      public static final double maxVelocity = 5.0;
      public static final double maxAcceleration = 3.0;

      public static final double translationalP = 0.75;
      public static final double translationalI = 0.0;
      public static final double translationalD = 0.0;

      public static final double rotationalP = 0.75;
      public static final double rotationalI = 0.0;
      public static final double rotationalD = 0.0;
    }

    public static class FrontLeftModule {
      public static final int driveID = 4;
      public static final int turnID = 8;
      public static final Rotation2d angleOffset = Rotation2d.fromDegrees(270);
    }

    public static class FrontRightModule {
      public static final int driveID = 2;
      public static final int turnID = 6;
      public static final Rotation2d angleOffset = Rotation2d.fromDegrees(0);
    }

    public static class BackLeftModule {
      public static final int driveID = 1;
      public static final int turnID = 5;
      public static final Rotation2d angleOffset = Rotation2d.fromDegrees(180);
    }

    public static class BackRightModule {
      public static final int driveID = 3;
      public static final int turnID = 7;
      public static final Rotation2d angleOffset = Rotation2d.fromDegrees(90);
    }
  }
}
