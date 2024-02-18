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
  public static int INTAKE_PORT = 16;
  public static int ELEVATOR_R = 17;
  public static int ELEVATOR_L = 18;
  public static int SHOOT_PORT_R = 19;
  public static int SHOOT_PORT_L = 20;

  public static final double LOOP_PERIOD_SECS = 0.02;

  public static final boolean TUNING_MODE = false;

  // An empty string uses the default CAN bus; specify the name of the CANivore as
  // appropriate
  public static final String CAN_BUS_NAME = "";

  // FIXME: specify the name of the camera used for detecting AprilTags
  public static final String CAMERA_NAME = "Front";

  private static final RobotType ROBOT = RobotType.ROBOT_2023_SEASON;

  private static final Alert invalidRobotAlert =
      new Alert("Invalid robot selected, using competition robot as default.", AlertType.ERROR);

  public static RobotType getRobot() {
    if (RobotBase.isReal()) {
      if (ROBOT == RobotType.ROBOT_SIMBOT) { // Invalid robot selected
        invalidRobotAlert.set(true);
        return RobotType.ROBOT_2023_SEASON;
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
      case ROBOT_2023_SEASON:
        return RobotBase.isReal() ? Mode.REAL : Mode.REPLAY;

      case ROBOT_SIMBOT:
        return Mode.SIM;

      default:
        return Mode.REAL;
    }
  }

  public enum RobotType {
    ROBOT_2023_SEASON,
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
  public static double INTAKE_SPEED = -0.1;

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
  public static double kB0 = 2000;

  public static double kB1 = 7000;
  public static double kB2 = 7000;
  public static double kB3 = 7000;
  public static double kB4 = 7000;
  public static double kB5 = 7000;
  public static double kB6 = 7000;
  public static double kB7 = 9300;
  public static double kB8 = 9500;
  public static double kB9 = 10000;
  public static double kB10 = 10500;
  public static double kB11 = 11000;
  public static double kB12 = 11500;
  public static double kB13 = 12000;
  public static double kB14 = 12500;
  public static double kB15 = 13000;
  public static double kB16 = 13500;
  public static double kB17 = 14000;
  public static double kB18 = 15000;
  public static double kB19 = 16000;
  public static double kB20 = 17000;
  public static double kB21 = 18000;
  public static double kB22 = 19000;
  public static double kB23 = 20000;
  public static double kB24 = 21000;
  public static double kB25 = 22000;
  public static double kT0 = 1000;
  public static double kT1 = 3000;
  public static double kT2 = 3000;
  public static double kT3 = 3000;
  public static double kT4 = 3000;
  public static double kT5 = 3000;
  public static double kT6 = 3000;
  public static double kT7 = 3000;
  ;
  public static double kT8 = 3450;
  public static double kT9 = 3600;
  public static double kT10 = 3750;
  public static double kT11 = 3900;
  public static double kT12 = 4050;
  public static double kT13 = 4200;
  public static double kT14 = 4350;
  public static double kT15 = 4500;
  public static double kT16 = 4650;
  public static double kT17 = 4800;
  public static double kT18 = 5200;
  public static double kT19 = 5600;
  public static double kT20 = 6000;
  public static double kT21 = 6400;
  public static double kT22 = 6800;
  public static double kT23 = 7200;
  public static double kT24 = 7600;
  public static double kT25 = 8000;
  //
  //
  public static double kB26 = 500;
  public static double kT26 = 1000;

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
