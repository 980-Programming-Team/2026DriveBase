package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.subsystems.Superstructure;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public class ScoringManager {
  private GenericHID leftController;
  private GenericHID rightController;
  private LoggableInputs leftCLoggableInputs;
  // Pathfinding commands
  private Command pathFindToBCL;
  private Command pathFindToBCR;
  private Command pathFindToBLL;
  private Command pathFindToBLR;
  private Command pathFindToFLL;
  private Command pathFindToFLR;
  private Command pathFindToFCL;
  private Command pathFindToFCR;
  private Command pathFindToFRL;
  private Command pathFindToFRR;
  private Command pathFindToBRL;
  private Command pathFindToBRR;

  private Superstructure superStructure;

  public ScoringManager(int leftPort, int rightPort, Superstructure superstructure) {
    leftController = new Joystick(leftPort);
    rightController = new Joystick(rightPort);
    superStructure = superstructure;
  }

  public GenericHID getLeftController() {
    return leftController;
  }

  public GenericHID getRightController() {
    return rightController;
  }

  // ^^ operator console uses two microcontrollers (each has 10 digital buttons)

  public void configureScoringButtons() {
    new JoystickButton(rightController, 7)
        .onTrue(new InstantCommand(() -> {}).ignoringDisable(true));

    new JoystickButton(rightController, 9)
        .onTrue(new InstantCommand(() -> {}).ignoringDisable(true));

    new JoystickButton(rightController, 4)
        .onTrue(new InstantCommand(() -> {}).ignoringDisable(true));

    new JoystickButton(rightController, 3)
        .onTrue(new InstantCommand(() -> {}).ignoringDisable(true));

    new JoystickButton(rightController, 2)
        .onTrue(new InstantCommand(() -> {}).ignoringDisable(true));

    new JoystickButton(rightController, 8)
        .onTrue(new InstantCommand(() -> {}).ignoringDisable(true));
  }

  int index = 0;

  public void configScoringPosButtons() {

    if (DriverStation.getAlliance().isPresent()) {
      if (DriverStation.getAlliance().get() == Alliance.Red) {

        // Creates positions for the operator console to work with red reef if driver station exists

        /* new JoystickButton(leftController, 2)
        .whileTrue(
            pathFindToBRR =
                AutoBuilder.pathfindToPose(
                    AllianceFlipUtil.apply(FieldConstants.Reef.centerFaces[11]),
                    RobotContainer.constraints,
                    0));*/
      } else {

        // Creates positions for the operator console to work with blue reef if driver station
        // exists

        /*  new JoystickButton(leftController, 2)
        .whileTrue(
            pathFindToBRR =
                AutoBuilder.pathfindToPose(
                    (FieldConstants.Reef.centerFaces[11]), RobotContainer.constraints, 0));*/
      }
    }
  }
}
