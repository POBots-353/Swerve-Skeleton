// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.Constants.SwerveConstants;
import frc.robot.util.CTREModuleState;
import frc.robot.util.Conversions;

/** Add your docs here. */
public class FalconSwerveModule implements SwerveModule {
  private TalonFX driveMotor;
  private TalonFX turnMotor;

  private CANCoder canEncoder;

  private Rotation2d angleOffset;

  private double prevVelocity = 0.0;

  private SimpleMotorFeedforward driveFeedforward = new SimpleMotorFeedforward(SwerveConstants.driveKs,
      SwerveConstants.driveKv, SwerveConstants.driveKa);

  public FalconSwerveModule(int driveID, int turnID, int encoderID, Rotation2d angleOffset) {
    driveMotor = new TalonFX(driveID);
    turnMotor = new TalonFX(turnID);

    canEncoder = new CANCoder(encoderID);

    this.angleOffset = angleOffset;

    configureDriveMotor();
    configureTurnMotor();
    configureAngleEncoder();

    resetToAbsolute();
  }

  private void configureDriveMotor() {
    driveMotor.config_kP(0, SwerveConstants.driveP);

    driveMotor.configPeakOutputForward(1.0);
    driveMotor.configPeakOutputReverse(-1.0);

    driveMotor.setInverted(SwerveConstants.driveMotorInverted);
  }

  private void configureTurnMotor() {
    turnMotor.configPeakOutputForward(1.0);
    turnMotor.configPeakOutputReverse(-1.0);

    turnMotor.setInverted(SwerveConstants.turnMotorInverted);

    turnMotor.config_kP(0, SwerveConstants.turnP);
    turnMotor.config_kD(0, SwerveConstants.turnD);
  }

  private void configureAngleEncoder() {
    canEncoder.configFactoryDefault();

    canEncoder.configAbsoluteSensorRange(AbsoluteSensorRange.Unsigned_0_to_360);
    canEncoder.configSensorInitializationStrategy(SensorInitializationStrategy.BootToAbsolutePosition);

    canEncoder.configSensorDirection(SwerveConstants.canCoderInverted);
  }

  private void resetToAbsolute() {
    Rotation2d position = Rotation2d.fromDegrees(canEncoder.getAbsolutePosition() - angleOffset.getDegrees());

    turnMotor.setSelectedSensorPosition(Conversions.degreesToFalcon(position.getDegrees()));
  }

  @Override
  public void setState(SwerveModuleState state) {
    SwerveModuleState optimizedState = CTREModuleState.optimize(state, getAngle());

    turnMotor.set(ControlMode.Position, Conversions.degreesToFalcon(optimizedState.angle.getDegrees()));

    double currentVelocity = optimizedState.speedMetersPerSecond;
    double feedForward = driveFeedforward.calculate(prevVelocity, currentVelocity, 0.020);

    driveMotor.set(ControlMode.Velocity, Conversions.mpsToFalcon(optimizedState.speedMetersPerSecond),
        DemandType.ArbitraryFeedForward, feedForward);

    prevVelocity = currentVelocity;
  }

  @Override
  public SwerveModulePosition getModulePosition() {
    return new SwerveModulePosition(Conversions.falconToMeters(driveMotor.getSelectedSensorPosition()), getAngle());
  }

  @Override
  public SwerveModuleState getModuleState() {
    return new SwerveModuleState(getVelocity(), getAngle());
  }

  @Override
  public double getVelocity() {
    return Conversions.falconToMPS(driveMotor.getSelectedSensorVelocity());
  }

  @Override
  public Rotation2d getAngle() {
    return Rotation2d.fromDegrees(Conversions.falconToDegrees(turnMotor.getSelectedSensorPosition()));
  }

  @Override
  public Rotation2d getAbsoluteAngle() {
    return Rotation2d.fromDegrees(canEncoder.getAbsolutePosition());
  }
}
