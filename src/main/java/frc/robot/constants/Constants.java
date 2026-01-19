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

package frc.robot.constants;

import edu.wpi.first.wpilibj.RobotBase;

/**
 * This class defines the runtime mode used by AdvantageKit. The mode is always "real" when running
 * on a roboRIO. Change the value of "simMode" to switch between "sim" (physics sim) and "replay"
 * (log replay from a file).
 */
public final class Constants {

  public static final boolean tuningMode = true;

  public static boolean disableHAL = false;

  public static void disableHAL() {
    disableHAL = true;
  }

  public static final Mode simMode = Mode.SIM;
  public static final Mode currentMode = RobotBase.isReal() ? Mode.REAL : simMode;

  public static final String kCANivore = "CANmeloAnthony";

  public static final boolean elevatorEnabled = true;
  public static final boolean armEnabled = true;
  public static final boolean funnelEnabled = true;
  public static final boolean climberEnabled = true;

  public static final boolean magneticLimitSwitchesEnabled = true;
  public static final boolean beamBreaksEnabled = false;

  public static final double NEO_FREE_SPEED = 6000.0; // TODO find the real free speed
  // LED Constants
  public static final int LED_NUM = 133; // TODO: Determine number of leds
  public static final int LED_BRIGHTNESS = 140;

  public class LaserCan {
    public static final int kLaserCan = 34;
  }

  public static enum Mode {
    /** Running on a real robot. */
    REAL,

    /** Running a physics simulator. */
    SIM,

    /** Replaying from a log file. */
    REPLAY
  }
}
