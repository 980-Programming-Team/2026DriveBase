// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.collector;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.constants.*;

public class Collector extends SubsystemBase {
  private SparkMax index;
  private SparkMax collect;

  private boolean override;


  /** Creates a new Collector. */
  public Collector() {
    collect = new SparkMax(Constants.Collector.kCollector, MotorType.kBrushless);
    override = false;
  }

  @Override
  public void periodic() {

  }

  public void intake() {
    collect.set(-.7);
  }

  public void outtake() {
    collect.set(.7);
  }

  public void index() {
    index.set(.5);
  }

  public void indexIntoShooter() {
    collect.set(-.7);
    index.set(.5);
  }

  public void indexIntoAmp() {
    collect.set(.7);
    index.set(.3);
  }

  public void off() {
    collect.set(0);
    index.set(0);
  }
}
