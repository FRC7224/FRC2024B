// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

/*
 * Initially from https://github.com/Mechanical-Advantage/RobotCode2022
 */

package frc.robot;

import edu.wpi.first.wpilibj.RobotBase;
import frc.lib.team6328.util.Alert;
import frc.lib.team6328.util.Alert.AlertType;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

  ///////////////// CAN Constants //////////////////////////////

  public static int CLIMB_PORT_R = 14;
  public static int CLIMB_PORT_L = 15;
  public static int ELEVATOR_R = 16;
  public static int ELEVATOR_L = 17;
  public static int SHOOT_PORT_R = 18;
  public static int SHOOT_PORT_L = 19;
  public static int INTAKE_PORT = 20;

  public static final double LOOP_PERIOD_SECS = 0.02;

  public static final boolean TUNING_MODE = false;

  // An empty string uses the default CAN bus; specify the name of the CANivore as
  // appropriate
  public static final String CAN_BUS_NAME = "";

  // FIXME: specify the name of the camera used for detecting AprilTags
  public static final String CAMERA_NAME = "Front";

  private static final RobotType ROBOT = RobotType.ROBOT_2024_SEASON;

  private static final Alert invalidRobotAlert =
      new Alert("Invalid robot selected, using competition robot as default.", AlertType.ERROR);

  public static RobotType getRobot() {
    if (RobotBase.isReal()) {
      if (ROBOT == RobotType.ROBOT_SIMBOT) { // Invalid robot selected
        invalidRobotAlert.set(true);
        return RobotType.ROBOT_2024_SEASON;
      } else {
        return ROBOT;
      }
    } else {
      return ROBOT;
    }
  }

  // update for various robots
  public static Mode getMode() {

    switch (getRobot()) {
      case ROBOT_2024_SEASON:
        return RobotBase.isReal() ? Mode.REAL : Mode.REPLAY;

      case ROBOT_SIMBOT:
        return Mode.SIM;

      default:
        return Mode.REAL;
    }
  }

  public enum RobotType {
    ROBOT_2024_SEASON,
    ROBOT_SIMBOT
  }

  public enum Mode {
    REAL,
    REPLAY,
    SIM
  }
  //////  autobalace  Constants ///////////////////////////

  public static double PITCH_LIMIT = 5.0;
  public static double PITCH_CAL_OFFSET = 0;

  ////// Intake Constants ///////////////////////////
  public static double INTAKE_TIMER = 3.0;
  public static double INTAKE_SPEED = -0.1;
  public static int NOTE_SENSOR_CHANNEL = 4;

  ////// Elevator Constants ///////////////////////////
  public static double ELEVATOR_SPEED = -0.1;

  ////// ARM Extend Constants ///////////////////////////

  public static double OV_ARM = 0.1;

  /**
   * Which PID slot to pull gains from. Starting 2018, you can choose from 0,1,2 or 3. Only the
   * first two (0,1) are visible in web-based configuration.
   */
  public static final int kSlotIdx = 0;

  /**
   * Talon SRX/ Victor SPX will supported multiple (cascaded) PID loops. For now we just want the
   * primary one.
   */
  public static final int kPIDLoopIdx = 0;

  /**
   * Set to zero to skip waiting for confirmation, set to nonzero to wait and report to DS if action
   * fails.
   */
  public static final int kTimeoutMs = 30;

  /* Choose so that Talon does not report sensor out of phase */
  public static boolean kSensorPhase = false;

  /** Choose based on what direction you want to be positive, this does not affect motor invert. */
  public static boolean kMotorInvert = false;

  /**
   * Gains used in Positon Closed Loop, to be adjusted accordingly Gains(kp, ki, kd, kf, izone, peak
   * output);
   */
  public static final Gains kGains = new Gains(0.5, 0.0, 0.1, 0.00, 0, 0.5);

  public static final double DEADBAND = 0.05;

  ////// Climb  Constants ///////////////////////////

  public static double CLIMB_MAX = 12.05 * 4096;
  public static double AUTO_LEVEL_PRESET = 12.05 * 4096;
  public static double OV_CLIMB = 0.1;

  /**
   * Which PID slot to pull gains from. Starting 2018, you can choose from 0,1,2 or 3. Only the
   * first two (0,1) are visible in web-based configuration.
   */
  public static final int kclimb1SlotIdx = 0;

  /** Talon FX supports multiple (cascaded) PID loops. For now we just want the primary one. */
  public static final int kclimb1PIDLoopIdx = 0;

  /**
   * Set to zero to skip waiting for confirmation, set to nonzero to wait and report to DS if action
   * fails.
   */
  public static final int kclimb1TimeoutMs = 30;

  /* Choose so that Talon does not report sensor out of phase */
  public static boolean kclimb1SensorPhase = true;

  /** Choose based on what direction you want to be positive, this does not affect motor invert. */
  public static boolean kclimb1MotorInvert = false;

  /**
   * Gains used in Positon Closed Loop, to be adjusted accordingly Gains(kp, ki, kd, kf, izone, peak
   * output);
   */
  public static final Gains kGainsR1 = new Gains(0.01, 0.0, 0.5, 0.0, 0, 1.0);

  ////// ARM Rotate 2 Constants ///////////////////////////

  /**
   * Which PID slot to pull gains from. Starting 2018, you can choose from 0,1,2 or 3. Only the
   * first two (0,1) are visible in web-based configuration.
   */
  public static final int kclimb2SlotIdx = 0;

  /** Talon FX supports multiple (cascaded) PID loops. For now we just want the primary one. */
  public static final int kclimb2PIDLoopIdx = 0;

  /**
   * Set to zero to skip waiting for confirmation, set to nonzero to wait and report to DS if action
   * fails.
   */
  public static final int kclimb2TimeoutMs = 30;

  /* Choose so that Talon does not report sensor out of phase */
  public static boolean kclimb2SensorPhase = true;

  /** Choose based on what direction you want to be positive, this does not affect motor invert. */
  public static boolean kclimb2MotorInvert = false;

  /**
   * Gains used in Positon Closed Loop, to be adjusted accordingly Gains(kp, ki, kd, kf, izone, peak
   * output);
   */
  public static final Gains kGainsR2 =
      new Gains(0.01, 0.0, 0.5, 0.0, 0, 1.0); // Move Ball to shooter

  public static int kballIntakeTimer_timer = 5;

  /** *** shoot constants */

  // Shooter / Elevator Constants
  public static int ZONE_LOW = 0;

  public static int ZONE_HIGH = 1;
  public static double kL0 = 250;
  public static double kL1 = 1500;

  public static double kR0 = 250;
  public static double kR1 = 1500;

  public static int kshortshootzone = 26;
  public static double kelvspeed = -0.7;

  public static double shooterTolerance = 300.0;
  public static double kshooterTimer_spin = 1.2; // / was 1.2
  public static double kshooterTimer_timer = 2.2; // / was 1.2
  public static boolean shooterMode = false;
  // public static int kPIDLoopIdx = 0;
  // public static int kTimeoutMs = 30;
  public static double kshootRightP = 0.25;
  public static double kshootRightI = 0.001;
  public static double kshootRightD = 20;
  public static double kshootRightF = 1023.0 / 7200.0;
  //
  public static double kshootLeftP = 0.25;
  public static double kshootLeftI = 0.001;
  public static double kshootLeftD = 20;
  public static double kshootLeftF = 1023.0 / 7200.0;
  public static int rightencoder;
  public static int leftencoder;
}
