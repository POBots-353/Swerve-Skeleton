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

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
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

  private CANCoder turnEncoder;

  private Rotation2d angleOffset;

  private SparkMaxPIDController drivePID;

  private SimpleMotorFeedforward driveFeedforward = new SimpleMotorFeedforward(SwerveConstants.driveKs,
      SwerveConstants.driveKv, SwerveConstants.driveKa);

  private PIDController turnPIDController = new PIDController(SwerveConstants.turnP, 0, SwerveConstants.turnD);

  private double prevVelocity = 0.0;

  public SparkMaxSwerveModule(int driveID, int turnID, int encoderID, Rotation2d angleOffset) {
    driveMotor = new CANSparkMax(driveID, MotorType.kBrushless);
    turnMotor = new CANSparkMax(turnID, MotorType.kBrushless);

    driveEncoder = driveMotor.getEncoder();

    turnEncoder = new CANCoder(encoderID);

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
    drivePID = driveMotor.getPIDController();

    drivePID.setP(SwerveConstants.driveP);
    drivePID.setOutputRange(-1, 1);

    driveEncoder.setPositionConversionFactor(SwerveConstants.drivePositionConversion);
    driveEncoder.setVelocityConversionFactor(SwerveConstants.driveVelocityConversion);

    driveMotor.setInverted(SwerveConstants.driveMotorInverted);
  }

  public void configureTurnMotor() {
    turnPIDController.enableContinuousInput(-180, 180);

    turnMotor.setInverted(SwerveConstants.turnMotorInverted);
  }

  private void configureAngleEncoder() {
    turnEncoder.configFactoryDefault();

    turnEncoder.configAbsoluteSensorRange(AbsoluteSensorRange.Unsigned_0_to_360);
    turnEncoder.configSensorInitializationStrategy(SensorInitializationStrategy.BootToAbsolutePosition);
    turnEncoder.configSensorDirection(SwerveConstants.canCoderInverted);
  }

  private void resetToAbsolute() {
    Rotation2d position = Rotation2d
        .fromRotations(turnEncoder.getAbsolutePosition() - angleOffset.getDegrees());

    turnEncoder.setPosition(position.getDegrees());
  }

  @Override
  public void setState(SwerveModuleState state) {
    SwerveModuleState optimizedState = SwerveModuleState.optimize(state, getAngle());

    turnMotor.set(turnPIDController.calculate(getAngle().getDegrees(), optimizedState.angle.getDegrees()));

    double currentVelocity = optimizedState.speedMetersPerSecond;
    double feedForward = driveFeedforward.calculate(prevVelocity, currentVelocity, 0.020);
    drivePID.setReference(optimizedState.speedMetersPerSecond, ControlType.kVelocity, 0, feedForward,
        ArbFFUnits.kVoltage);

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
    return Rotation2d.fromRotations(turnEncoder.getPosition());
  }

  @Override
  public Rotation2d getAbsoluteAngle() {
    return Rotation2d.fromDegrees(turnEncoder.getAbsolutePosition());
  }

  @Override
  public double getTurnRotations() {
    return getAngle().getRotations();
  }

  @Override
  public double getTurnDegrees() {
    return getAngle().getDegrees();
  }

  @Override
  public double getAbsoluteTurnDegrees() {
    return MathUtil.inputModulus(getAbsoluteAngle().getDegrees(), 0, 360);
  }
}
