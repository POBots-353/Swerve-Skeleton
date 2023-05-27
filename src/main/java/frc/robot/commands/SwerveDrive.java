// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.SwerveConstants;
import frc.robot.subsystems.Swerve;

public class SwerveDrive extends CommandBase {
  private DoubleSupplier strafeSpeed;
  private DoubleSupplier forwardSpeed;
  private DoubleSupplier angleSpeed;

  private double maxTranslationalSpeed;
  private double maxAngularSpeed;

  private Swerve swerve;

  /** Creates a new SwerveDrive. */
  public SwerveDrive(DoubleSupplier forwardSpeedSupplier, DoubleSupplier strafeSpeedSupplier,
      DoubleSupplier angleSpeedSupplier, double maxTranslationalSpeed, double maxAngularSpeed, Swerve swerve) {
    this.forwardSpeed = forwardSpeedSupplier;
    this.strafeSpeed = strafeSpeedSupplier;
    this.angleSpeed = angleSpeedSupplier;

    this.maxTranslationalSpeed = maxTranslationalSpeed;
    this.maxAngularSpeed = maxAngularSpeed;

    this.swerve = swerve;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(swerve);
  }

  public SwerveDrive(DoubleSupplier forwardSpeedSupplier, DoubleSupplier strafeSpeedSupplier,
      DoubleSupplier angleSpeedSupplier, Swerve swerve) {
    this(forwardSpeedSupplier, strafeSpeedSupplier, angleSpeedSupplier, SwerveConstants.maxTranslationalSpeed,
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
    double angularRadiansPerSecond = angleSpeed.getAsDouble() * maxAngularSpeed;

    swerve.driveFieldOriented(forwardMetersPerSecond, strafeMetersPerSecond, angularRadiansPerSecond);
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
