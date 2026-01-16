package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

// import frc.robot.subsystems.climber.Climber;
public class Superstructure extends SubsystemBase {
  // revTime
  private Timer feedSetTimer;

  private DutyCycleEncoder armEncoder;

  public Superstructure() {
    feedSetTimer = new Timer();
    feedSetTimer.stop();
    feedSetTimer.reset();
  }

  @Override
  public void periodic() {}
}
