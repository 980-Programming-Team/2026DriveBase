package frc.robot.subsystems.shooter;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.*;

// shooter goal, adjust angle up down smoothly based off april tag reading,
public class Shooter extends SubsystemBase {

  private SparkMax shooterSpark;

  private final double SHOOTER_RPM_MAX = 5800; // /60 to make rpm to rps,  TODO: get actual max RPM

  private RelativeEncoder shooterMotor;

  private PIDController shooterController;

  private double shooterSpeed;
  private boolean reachedMaxSpeed;

  public Shooter() {
    shooterSpark = new SparkMax(Constants.Manipulator.kShooter, MotorType.kBrushless);

    shooterMotor = shooterSpark.getEncoder();
    shooterController = new PIDController(.001, 0, 0);

    shooterSpeed = 0;
    reachedMaxSpeed = false;
  }

  @Override
  public void periodic() {
    shooterSpeed = shooterMotor.getVelocity();
  }

  public void stopShooter() {
    shooterSpark.setVoltage(0);
  }

  public void fireBall(double upper, double angularPosition) {
    double setpoint = upper; // <30  2000 4000

    shooterSpark.setVoltage(setpoint);
    shooterSpark.setVoltage(shooterController.calculate(shooterSpeed, setpoint));
  }
}
