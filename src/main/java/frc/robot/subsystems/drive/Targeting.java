package frc.robot.subsystems.drive;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.*;
import frc.robot.subsystems.vision.*;
import java.util.HashMap;

public class Targeting extends SubsystemBase {

  private final SwerveDrivePoseEstimator posEstimator;
  private final Translation2d hubCenter =
      FieldConstants.Hub.center.getTranslation();

  public Targeting(SwerveDrivePoseEstimator posEstimator) {
    this.posEstimator = posEstimator;
  }

  public Rotation2d getRotationErrorToHub(double offsetRadians) {
    Pose2d pose = posEstimator.getEstimatedPosition();

    Translation2d robotPos = pose.getTranslation();
    Rotation2d robotHeading = pose.getRotation();

    Translation2d toHub = hubCenter.minus(robotPos);

    Rotation2d desiredHeading =
        new Rotation2d(Math.atan2(toHub.getY(), toHub.getX()))
            .plus(Rotation2d.fromRadians(offsetRadians));

    return desiredHeading.minus(robotHeading);
  }
}
