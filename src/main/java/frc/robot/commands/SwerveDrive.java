// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.SwerveConstants;
import frc.robot.subsystems.Swerve;

public class SwerveDrive extends CommandBase {
  private DoubleSupplier strafeSpeed;
  private DoubleSupplier forwardSpeed;
  private DoubleSupplier angleX;
  private DoubleSupplier angleY;
  private BooleanSupplier turnToAngle;

  private double maxTranslationalSpeed;
  private double maxAngularSpeed;

  private Swerve swerve;

  private PIDController turnToAngleController = new PIDController(SwerveConstants.headingKP, 0,
      SwerveConstants.headingKD);

  private final boolean isOpenLoop = true;

  /** Creates a new SwerveDrive. */
  public SwerveDrive(DoubleSupplier forwardSpeedSupplier, DoubleSupplier strafeSpeedSupplier,
      DoubleSupplier angleXSupplier, DoubleSupplier angleYSupplier, BooleanSupplier turnToAngleSupplier,
      double maxTranslationalSpeed, double maxAngularSpeed, Swerve swerve) {
    this.forwardSpeed = forwardSpeedSupplier;
    this.strafeSpeed = strafeSpeedSupplier;
    this.angleX = angleXSupplier;
    this.angleY = angleYSupplier;
    this.turnToAngle = turnToAngleSupplier;

    this.maxTranslationalSpeed = maxTranslationalSpeed;
    this.maxAngularSpeed = maxAngularSpeed;

    this.swerve = swerve;

    turnToAngleController.enableContinuousInput(-Math.PI, Math.PI);

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(swerve);
  }

  public SwerveDrive(DoubleSupplier forwardSpeedSupplier, DoubleSupplier strafeSpeedSupplier,
      DoubleSupplier angleXSupplier, Swerve swerve) {
    this(forwardSpeedSupplier, strafeSpeedSupplier, angleXSupplier, () -> 0.0, () -> false,
        SwerveConstants.maxTranslationalSpeed,
        SwerveConstants.maxAngularSpeed, swerve);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double strafeMetersPerSecond = strafeSpeed.getAsDouble() * maxTranslationalSpeed;
    double forwardMetersPerSecond = -forwardSpeed.getAsDouble() * maxTranslationalSpeed;
    double angleXComponent = angleX.getAsDouble();
    double angleYComponent = -angleY.getAsDouble();

    if (!turnToAngle.getAsBoolean()) {
      swerve.driveFieldOriented(forwardMetersPerSecond, strafeMetersPerSecond, angleXComponent * maxAngularSpeed,
          isOpenLoop);
    } else {
      Rotation2d desiredAngle = new Rotation2d(angleXComponent, angleYComponent);

      double angularSpeed = turnToAngleController.calculate(swerve.getRotation().getRadians(),
          desiredAngle.getRadians()) * maxAngularSpeed;

      swerve.driveFieldOriented(forwardMetersPerSecond, strafeMetersPerSecond, angularSpeed, isOpenLoop);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
