package frc.robot.subsystems.hood;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import frc.robot.constants.Constants;

public class Hood {

  private SparkMax angleSpark;
  private RelativeEncoder angleMotor;
  private PIDController angleController;
  private final double SHOOTER_RPM_MAX = 5800;

  private ArmFeedforward ffShooter;
  private DutyCycleEncoder throughBoreEncoder;

  private final double SHOOTER_FEEDFORWARD_KS = 0.05; // probably the same as before
  private final double SHOOTER_FEEDFORWARD_KV = 12.0 / SHOOTER_RPM_MAX;
  private final double SHOOTER_FEEDFORWARD_KG = 2.0; // adjust

  public Hood() {
    angleSpark = new SparkMax(Constants.Manipulator.kangleMotor, MotorType.kBrushless);

    angleMotor = angleSpark.getEncoder();
    angleController = new PIDController(.002, 0, 0);

    ffShooter =
        new ArmFeedforward(SHOOTER_FEEDFORWARD_KS, SHOOTER_FEEDFORWARD_KS, SHOOTER_FEEDFORWARD_KV);

    throughBoreEncoder = new DutyCycleEncoder(5, 250.0, 0.0);
    throughBoreEncoder.setInverted(true);
  }

  public void angleHood(double angularPosition) {
    double armPosition = angularPosition;
    ffShooter.calculate(armPosition, armPosition);
  }
}
