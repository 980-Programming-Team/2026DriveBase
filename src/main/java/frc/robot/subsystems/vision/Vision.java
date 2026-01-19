// Copyright 2021-2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot.subsystems.vision;

import static frc.robot.subsystems.vision.VisionConstants.*;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.vision.VisionIO.PoseObservationType;
import java.util.LinkedList;
import java.util.List;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class Vision extends SubsystemBase {
  private final VisionConsumer consumer;
  private final VisionIO[] io;
  private final VisionIOInputsAutoLogged[] inputs;
  private final Alert[] disconnectedAlerts;
  private final Supplier<Pose2d> groundTruthSupplier; // optional; used for sim fallback

  // Backwards-compatible constructor (no ground-truth supplier)
  public Vision(VisionConsumer consumer, VisionIO... io) {
    this(consumer, null, io);
  }

  // New constructor accepts optional ground-truth supplier for simulation
  public Vision(VisionConsumer consumer, Supplier<Pose2d> groundTruthSupplier, VisionIO... io) {
    this.consumer = consumer;
    this.groundTruthSupplier = groundTruthSupplier;
    this.io = io;

    // Initialize inputs
    this.inputs = new VisionIOInputsAutoLogged[io.length];
    for (int i = 0; i < inputs.length; i++) {
      inputs[i] = new VisionIOInputsAutoLogged();
    }

    // Initialize disconnected alerts
    this.disconnectedAlerts = new Alert[io.length];
    // Use io.length here so the number of alerts matches the provided IO devices
    for (int i = 0; i < io.length; i++) {
      disconnectedAlerts[i] =
          new Alert(
              "Vision camera " + Integer.toString(i) + " is disconnected.", AlertType.kWarning);
    }
  }

  /**
   * Returns the X angle to the best target, which can be used for simple servoing with vision.
   *
   * @param cameraIndex The index of the camera to use.
   */
  public Rotation2d getTargetX(int cameraIndex) {
    // Safely handle invalid indices and missing observations to avoid NPE/IOOBE.
    if (cameraIndex < 0 || cameraIndex >= inputs.length) {
      return new Rotation2d();
    }
    var obs = inputs[cameraIndex].latestTargetObservation;
    if (obs == null) {
      return new Rotation2d();
    }
    return obs.tx();
  }

  public void periodic() {
    for (int i = 0; i < io.length; i++) {
      io[i].updateInputs(inputs[i]);
      Logger.processInputs("Vision/Camera" + Integer.toString(i), inputs[i]);
    }

    // Initialize logging values
    List<Pose3d> allTagPoses = new LinkedList<>();
    List<Pose3d> allRobotPoses = new LinkedList<>();
    List<Pose3d> allRobotPosesAccepted = new LinkedList<>();
    List<Pose3d> allRobotPosesRejected = new LinkedList<>();

    // If there are no configured IO devices but we have a ground-truth supplier, generate
    // simulated vision measurements for two virtual Limelight-like cameras. Populate per-camera
    // logs so Advantage Scope shows camera and summary data.
    if (io.length == 0 && groundTruthSupplier != null) {
      Pose2d truth = groundTruthSupplier.get();
      double timestamp = Timer.getFPGATimestamp();

      // Number of simulated cameras (two Limelight4s)
      final int simulatedCameras = 2;
      for (int cam = 0; cam < simulatedCameras; cam++) {
        // Slightly vary stddev per camera to simulate different mounting/noise characteristics
        double linearStd = 0.05 + cam * 0.02;
        double angularStd = 0.02 + cam * 0.01;
        // Stagger timestamps slightly to emulate different capture times
        double camTimestamp = timestamp - cam * 0.005;

        // Forward the simulated observation to the drivetrain pose estimator
        consumer.accept(truth, camTimestamp, VecBuilder.fill(linearStd, linearStd, angularStd));

        // Build Pose3d for logging (z = 0.0 for flat field; use yaw from Pose2d)
        Pose3d robotPose3d =
            new Pose3d(
                truth.getX(),
                truth.getY(),
                0.0,
                new Rotation3d(0.0, 0.0, truth.getRotation().getRadians()));

        // Populate tag poses from the apriltag layout for camera logs
        List<Pose3d> tagPoses = new LinkedList<>();
        // aprilTagLayout.getTags() returns a List<AprilTag>; extract each tag's pose
        for (var aprilTag : aprilTagLayout.getTags()) {
          // AprilTag exposes its pose as the 'pose' field
          tagPoses.add(aprilTag.pose);
        }

        // Per-camera robot pose lists (accepted)
        List<Pose3d> robotPoses = new LinkedList<>();
        robotPoses.add(robotPose3d);
        List<Pose3d> robotPosesAccepted = new LinkedList<>(robotPoses);

        // Record camera-level outputs so Advantage Scope shows them
        Logger.recordOutput(
            "Vision/Camera" + Integer.toString(cam) + "/TagPoses", tagPoses.toArray(new Pose3d[0]));
        Logger.recordOutput(
            "Vision/Camera" + Integer.toString(cam) + "/RobotPoses",
            robotPoses.toArray(new Pose3d[0]));
        Logger.recordOutput(
            "Vision/Camera" + Integer.toString(cam) + "/RobotPosesAccepted",
            robotPosesAccepted.toArray(new Pose3d[0]));

        // Add to summary collections
        allTagPoses.addAll(tagPoses);
        allRobotPoses.addAll(robotPoses);
        allRobotPosesAccepted.addAll(robotPosesAccepted);
      }

      // Log the generated ground-truth pose for debugging
      Logger.recordOutput("Vision/Sim/GeneratedPose", truth);
    }

    // Loop over cameras
    for (int cameraIndex = 0; cameraIndex < io.length; cameraIndex++) {
      // Update disconnected alert
      disconnectedAlerts[cameraIndex].set(!inputs[cameraIndex].connected);

      // Initialize logging values
      List<Pose3d> tagPoses = new LinkedList<>();
      List<Pose3d> robotPoses = new LinkedList<>();
      List<Pose3d> robotPosesAccepted = new LinkedList<>();
      List<Pose3d> robotPosesRejected = new LinkedList<>();

      // Add tag poses
      for (int tagId : inputs[cameraIndex].tagIds) {
        var tagPose = aprilTagLayout.getTagPose(tagId);
        if (tagPose.isPresent()) {
          tagPoses.add(tagPose.get());
        }
      }

      // Loop over pose observations
      for (var observation : inputs[cameraIndex].poseObservations) {
        // Check whether to reject pose
        boolean rejectPose =
            observation.tagCount() == 0 // Must have at least one tag
                || (observation.tagCount() == 1
                    && observation.ambiguity() > maxAmbiguity) // Cannot be high ambiguity
                || Math.abs(observation.pose().getZ())
                    > maxZError // Must have realistic Z coordinate

                // Must be within the field boundaries
                || observation.pose().getX() < 0.0
                || observation.pose().getX() > aprilTagLayout.getFieldLength()
                || observation.pose().getY() < 0.0
                || observation.pose().getY() > aprilTagLayout.getFieldWidth();
        // || observation.type().equals(PoseObservationType.MEGATAG_1);

        // Add pose to log
        robotPoses.add(observation.pose());
        if (rejectPose) {
          robotPosesRejected.add(observation.pose());
        } else {
          robotPosesAccepted.add(observation.pose());
        }

        // Skip if rejected
        if (rejectPose) {
          continue;
        }

        // Calculate standard deviations
        double stdDevFactor =
            Math.pow(observation.averageTagDistance(), 2.0) / observation.tagCount();
        double linearStdDev = linearStdDevBaseline * stdDevFactor;
        double angularStdDev = angularStdDevBaseline * stdDevFactor;
        if (observation.type() == PoseObservationType.MEGATAG_2) {
          linearStdDev *= linearStdDevMegatag2Factor;
          angularStdDev *= angularStdDevMegatag2Factor;
        }
        if (cameraIndex < cameraStdDevFactors.length) {
          linearStdDev *= cameraStdDevFactors[cameraIndex];
          angularStdDev *= cameraStdDevFactors[cameraIndex];
        }

        // Send vision observation
        consumer.accept(
            observation.pose().toPose2d(),
            observation.timestamp(),
            VecBuilder.fill(linearStdDev, linearStdDev, angularStdDev));
      }

      // Log camera datadata
      Logger.recordOutput(
          "Vision/Camera" + Integer.toString(cameraIndex) + "/TagPoses",
          tagPoses.toArray(new Pose3d[tagPoses.size()]));
      Logger.recordOutput(
          "Vision/Camera" + Integer.toString(cameraIndex) + "/RobotPoses",
          robotPoses.toArray(new Pose3d[robotPoses.size()]));
      Logger.recordOutput(
          "Vision/Camera" + Integer.toString(cameraIndex) + "/RobotPosesAccepted",
          robotPosesAccepted.toArray(new Pose3d[robotPosesAccepted.size()]));
      Logger.recordOutput(
          "Vision/Camera" + Integer.toString(cameraIndex) + "/RobotPosesRejected",
          robotPosesRejected.toArray(new Pose3d[robotPosesRejected.size()]));
      allTagPoses.addAll(tagPoses);
      allRobotPoses.addAll(robotPoses);
      allRobotPosesAccepted.addAll(robotPosesAccepted);
      allRobotPosesRejected.addAll(robotPosesRejected);
    }

    // Log summary data
    Logger.recordOutput(
        "Vision/Summary/TagPoses", allTagPoses.toArray(new Pose3d[allTagPoses.size()]));
    Logger.recordOutput(
        "Vision/Summary/RobotPoses", allRobotPoses.toArray(new Pose3d[allRobotPoses.size()]));
    Logger.recordOutput(
        "Vision/Summary/RobotPosesAccepted",
        allRobotPosesAccepted.toArray(new Pose3d[allRobotPosesAccepted.size()]));
    Logger.recordOutput(
        "Vision/Summary/RobotPosesRejected",
        allRobotPosesRejected.toArray(new Pose3d[allRobotPosesRejected.size()]));
  }

  @FunctionalInterface
  public static interface VisionConsumer {
    public void accept(
        Pose2d visionRobotPoseMeters,
        double timestampSeconds,
        Matrix<N3, N1> visionMeasurementStdDevs);
  }
}
