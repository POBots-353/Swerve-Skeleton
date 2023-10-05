package frc.robot.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;

public class GeometryUtil {
  /**
   * Logical inverse of the Pose exponential, written by Team 254
   * https://github.com/Team254/FRC-2022-Public/blob/main/src/main/java/com/team254/lib/geometry/Pose2d.java
   * 
   * @param transform Pose to perform the log on
   * @return {@link Twist2d} of the transformed pose.
   */
  public static Twist2d poseLog(Pose2d transform) {
    final double dtheta = transform.getRotation().getRadians();
    final double halfDtheta = dtheta / 2.0;

    final double cosMinusOne = transform.getRotation().getCos() - 1;

    double halfThetaByTanOfHalfDtheta;
    if (Math.abs(cosMinusOne) < 1E-9) {
      halfThetaByTanOfHalfDtheta = 1.0 - 1.0 / 12.0 * dtheta * dtheta;
    } else {
      halfThetaByTanOfHalfDtheta = -(halfDtheta * transform.getRotation().getSin()) / cosMinusOne;
    }

    Translation2d translationPart = transform
        .getTranslation()
        .rotateBy(new Rotation2d(halfThetaByTanOfHalfDtheta, -halfDtheta));

    return new Twist2d(translationPart.getX(), translationPart.getY(), dtheta);
  }
}
