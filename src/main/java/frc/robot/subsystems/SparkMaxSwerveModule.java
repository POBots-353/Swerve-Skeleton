// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxAbsoluteEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;
import com.revrobotics.SparkMaxPIDController.ArbFFUnits;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.Constants.SwerveConstants;

/** Add your docs here. */
public class SparkMaxSwerveModule implements SwerveModule {
  private CANSparkMax driveMotor;
  private CANSparkMax turnMotor;

  private RelativeEncoder driveEncoder;
  private SparkMaxAbsoluteEncoder turnEncoder;

  private SparkMaxPIDController drivePID;

  private SimpleMotorFeedforward driveFeedforward = new SimpleMotorFeedforward(SwerveConstants.driveKs,
      SwerveConstants.driveKv, SwerveConstants.driveKa);

  private SparkMaxPIDController turnPID;

  private double prevVelocity = 0.0;

  public SparkMaxSwerveModule(int driveID, int turnID, Rotation2d angleOffset) {
    driveMotor = new CANSparkMax(driveID, MotorType.kBrushless);
    turnMotor = new CANSparkMax(turnID, MotorType.kBrushless);

    driveEncoder = driveMotor.getEncoder();
    turnEncoder = turnMotor.getAbsoluteEncoder(Type.kDutyCycle);

    configureDriveMotor();
    configureTurnMotor(angleOffset);
  }

  public SparkMaxSwerveModule(int driveID, int turnID) {
    this(driveID, turnID, new Rotation2d(0));
  }

  public void configureDriveMotor() {
    drivePID = driveMotor.getPIDController();

    drivePID.setP(SwerveConstants.driveP);
    drivePID.setOutputRange(-1, 1);

    driveEncoder.setPositionConversionFactor(SwerveConstants.drivePositionConversion);
    driveEncoder.setVelocityConversionFactor(SwerveConstants.driveVelocityConversion);
  }

  public void configureTurnMotor(Rotation2d angleOffset) {
    turnPID = turnMotor.getPIDController();

    turnPID.setP(SwerveConstants.turnP);
    turnPID.setD(SwerveConstants.turnD);
    turnPID.setFeedbackDevice(turnEncoder);
    turnPID.setOutputRange(-1, 1);

    turnPID.setPositionPIDWrappingEnabled(true);
    turnPID.setPositionPIDWrappingMinInput(0.0);
    turnPID.setPositionPIDWrappingMaxInput(1.0);

    turnEncoder.setInverted(true);
    turnEncoder.setZeroOffset(angleOffset.getRotations());
  }

  @Override
  public void setState(SwerveModuleState state) {
    turnPID.setReference(state.angle.getRotations(), ControlType.kPosition);

    double currentVelocity = driveEncoder.getVelocity();
    double feedForward = driveFeedforward.calculate(prevVelocity, currentVelocity, 0.020);
    drivePID.setReference(state.speedMetersPerSecond, ControlType.kVelocity, 0, feedForward, ArbFFUnits.kVoltage);

    prevVelocity = currentVelocity;
  }

  @Override
  public SwerveModulePosition getModulePosition() {
    return new SwerveModulePosition(driveEncoder.getPosition(),
        Rotation2d.fromRotations(turnEncoder.getPosition()));
  }

  @Override
  public SwerveModuleState getModuleState() {
    return new SwerveModuleState(driveEncoder.getVelocity(), Rotation2d.fromRotations(turnEncoder.getPosition()));
  }

  @Override
  public double getVelocity() {
    return driveEncoder.getVelocity();
  }

  @Override
  public Rotation2d getAngle() {
    return Rotation2d.fromRotations(turnEncoder.getPosition());
  }

  @Override
  public double getTurnRotations() {
    return getAngle().getRotations();
  }

  @Override
  public double getTurnDegrees() {
    return getAngle().getDegrees();
  }
}
