package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

public interface SwerveModule {
  public default void setState(SwerveModuleState state) {
    setState(state, false);
  }

  public void setState(SwerveModuleState state, boolean isOpenLoop);

  public SwerveModulePosition getModulePosition();

  public SwerveModuleState getModuleState();

  public double getVelocity();

  public Rotation2d getAngle();

  public Rotation2d getAbsoluteAngle();
}
