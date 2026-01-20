package frc.robot.subsystems.drive;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;

public class RotateCommand extends Command {

  private final Drive drive;
  private final Targeting targeting;
  private final PIDController thetaController;
  private final double offsetRadians;

  public RotateCommand(
      Drive drive,
      Targeting targeting,
      double offsetRadians) {

    this.drive = drive;
    this.targeting = targeting;
    this.offsetRadians = offsetRadians;

    thetaController = new PIDController(
        5.0,  // kP (start here)
        0.0,
        0.35  // kD
    );

    thetaController.enableContinuousInput(-Math.PI, Math.PI);
    thetaController.setTolerance(Math.toRadians(1.5));

    addRequirements(drive);
  }

  @Override
  public void initialize() {
    thetaController.reset();
  }

  @Override
  public void execute() {
    Rotation2d error =
        targeting.getRotationErrorToHub(offsetRadians);

    double omega =
        thetaController.calculate(error.getRadians(), 0.0);

    // Clamp angular speed to robot limits
    omega = Math.max(
        -Drive.getMaxAngularSpeedRadPerSec(),
        Math.min(omega, Drive.getMaxAngularSpeedRadPerSec())
    );

    // ONLY control rotation, translation stays driver-controlled
    drive.runVelocity(0.0, 0.0, omega, true);
  }

  @Override
  public boolean isFinished() {
    return thetaController.atSetpoint();
  }

  @Override
  public void end(boolean interrupted) {
    drive.runVelocity(0.0, 0.0, 0.0, true);
  }
}
