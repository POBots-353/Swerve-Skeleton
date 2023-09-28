// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
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
  private RelativeEncoder turnEncoder;

  private CANCoder canEncoder;

  private Rotation2d angleOffset;

  private SparkMaxPIDController drivePID;
  private SparkMaxPIDController turnPID;

  private SimpleMotorFeedforward driveFeedforward = new SimpleMotorFeedforward(SwerveConstants.driveKs,
      SwerveConstants.driveKv, SwerveConstants.driveKa);

  private double prevVelocity = 0.0;

  public SparkMaxSwerveModule(int driveID, int turnID, int encoderID, Rotation2d angleOffset) {
    driveMotor = new CANSparkMax(driveID, MotorType.kBrushless);
    turnMotor = new CANSparkMax(turnID, MotorType.kBrushless);

    driveEncoder = driveMotor.getEncoder();
    turnEncoder = turnMotor.getEncoder();

    canEncoder = new CANCoder(encoderID);

    this.angleOffset = angleOffset;

    configureDriveMotor();
    configureTurnMotor();
    configureAngleEncoder();

    resetToAbsolute();
  }

  public SparkMaxSwerveModule(int driveID, int turnID, int encoderID) {
    this(driveID, turnID, encoderID, new Rotation2d(0));
  }

  public void configureDriveMotor() {
    driveMotor.setInverted(SwerveConstants.driveMotorInverted);

    driveMotor.setOpenLoopRampRate(SwerveConstants.openLoopRamp);
    driveMotor.setClosedLoopRampRate(SwerveConstants.closedLoopRamp);

    driveMotor.enableVoltageCompensation(SwerveConstants.voltageCompensation);
    driveMotor.setSmartCurrentLimit(SwerveConstants.driveCurrentLimit);

    drivePID = driveMotor.getPIDController();

    drivePID.setP(SwerveConstants.driveP);
    drivePID.setOutputRange(-1, 1);

    driveEncoder.setPositionConversionFactor(SwerveConstants.drivePositionConversion);
    driveEncoder.setVelocityConversionFactor(SwerveConstants.driveVelocityConversion);
  }

  public void configureTurnMotor() {
    turnMotor.setInverted(SwerveConstants.turnMotorInverted);

    turnMotor.enableVoltageCompensation(SwerveConstants.voltageCompensation);
    turnMotor.setSmartCurrentLimit(SwerveConstants.turnCurrentLimit);

    turnEncoder.setPositionConversionFactor(SwerveConstants.turnPositionConversion);

    turnPID = turnMotor.getPIDController();

    turnPID.setP(SwerveConstants.turnP);
    turnPID.setD(SwerveConstants.turnD);
    turnPID.setOutputRange(-1.0, 1.0);

    turnPID.setPositionPIDWrappingEnabled(true);
    turnPID.setPositionPIDWrappingMinInput(-Math.PI);
    turnPID.setPositionPIDWrappingMaxInput(Math.PI);
  }

  private void configureAngleEncoder() {
    canEncoder.configFactoryDefault();

    canEncoder.configAbsoluteSensorRange(AbsoluteSensorRange.Unsigned_0_to_360);
    canEncoder.configSensorInitializationStrategy(SensorInitializationStrategy.BootToAbsolutePosition);
    canEncoder.configSensorDirection(SwerveConstants.canCoderInverted);
  }

  private void resetToAbsolute() {
    Rotation2d position = Rotation2d
        .fromDegrees(canEncoder.getAbsolutePosition() - angleOffset.getDegrees());

    turnEncoder.setPosition(position.getRadians());
  }

  @Override
  public void setState(SwerveModuleState state, boolean isOpenLoop) {
    SwerveModuleState optimizedState = SwerveModuleState.optimize(state, getAngle());

    turnPID.setReference(optimizedState.angle.getRadians(), ControlType.kPosition);

    double currentVelocity = optimizedState.speedMetersPerSecond;

    if (isOpenLoop) {
      driveMotor.set(currentVelocity / SwerveConstants.maxModuleSpeed);
    } else {
      double feedForward = driveFeedforward.calculate(prevVelocity, currentVelocity, 0.020);

      drivePID.setReference(optimizedState.speedMetersPerSecond, ControlType.kVelocity, 0, feedForward,
          ArbFFUnits.kVoltage);
    }

    prevVelocity = currentVelocity;
  }

  @Override
  public SwerveModulePosition getModulePosition() {
    return new SwerveModulePosition(driveEncoder.getPosition(), getAngle());
  }

  @Override
  public SwerveModuleState getModuleState() {
    return new SwerveModuleState(getVelocity(), getAngle());
  }

  @Override
  public double getVelocity() {
    return driveEncoder.getVelocity();
  }

  @Override
  public Rotation2d getAngle() {
    return Rotation2d.fromRadians(turnEncoder.getPosition());
  }

  @Override
  public Rotation2d getAbsoluteAngle() {
    return Rotation2d.fromDegrees(canEncoder.getAbsolutePosition());
  }
}
