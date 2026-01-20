package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.collector.*;
import frc.robot.subsystems.drive.*;
import frc.robot.subsystems.shooter.*;

// import frc.robot.subsystems.climber.Climber;
public class Superstructure extends SubsystemBase {
  // revTime
  private Timer feedSetTimer;

  private DutyCycleEncoder armEncoder;

  public Targeting targeting;
  public Shooter shooter;
  public Collector intake;

  public Superstructure(Targeting target, Shooter shooter, Collector collector) {
    feedSetTimer = new Timer();
    feedSetTimer.stop();
    feedSetTimer.reset();
    targeting = target;
    
    this.shooter = shooter;
    this.intake = collector;
  }

  @Override
  public void periodic() {}
}
