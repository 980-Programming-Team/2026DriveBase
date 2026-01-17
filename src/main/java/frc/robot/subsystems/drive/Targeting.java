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
  // only do when moving and set up first version
  private NetworkTable limelight;
  private NetworkTableEntry tv;
  private NetworkTableEntry tx;
  private NetworkTableEntry ty;

  private double validTarget;
  private double x;
  private double y;
  // position, velocity or angle in radians respectively
  private HashMap<Double, Double> firingTable = new HashMap<Double, Double>();
  private HashMap<Double, Double> angleTable = new HashMap<Double, Double>();
  SwerveDrivePoseEstimator posEstimator;
  Pose2d pose2d;
  Translation2d translation2d;
  Rotation2d rotation2d;
  Translation2d hubCenter = FieldConstants.Hub.center.getTranslation();

  private VisionIOLimelight visionIOInputsLimelight;

  public Targeting(VisionIOLimelight visionIOLimelight, SwerveDrivePoseEstimator pos) {
    posEstimator = pos;
    this.visionIOInputsLimelight = visionIOLimelight;
    pose2d = new Pose2d();
    rotation2d = new Rotation2d();
    translation2d = new Translation2d();
  }

  @Override
  public void periodic() {
    pose2d = posEstimator.getEstimatedPosition();
    translation2d = pose2d.getTranslation();
    rotation2d = pose2d.getRotation();
  }

  private void RotateRobot(Pose2d aprilTag) {
    Translation2d toTarget = hubCenter.minus(translation2d);
    Rotation2d rotationToFace = new Rotation2d(Math.atan2(toTarget.getY(), toTarget.getX()));
  }
}
