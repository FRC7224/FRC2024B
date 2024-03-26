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

  //////  autobalace  Constants ///////////////////////////

  public static double PITCH_CAL_OFFSET = 0;
  public static double ROLL_CAL_OFFSET = 0;
  public static double ROLL_OFFSET = 0;

  ////// Intake Constants ///////////////////////////
  public static double INTAKE_TIMER = 3.0;
  public static double INTAKE_SPEED = -0.7;
  public static int NOTE_SENSOR_CHANNEL = 4;

  ////// Elevator Constants ///////////////////////////
  public static double ELEVATOR_SPEED = -0.14;
  public static double ELEVATOR_SPEED_SHOOT = -1.0;

  ////// Climb Constants ///////////////////////////
  public static double CLIMB_MAX_HEIGHT = 660000;
  public static double CLIMB_MIN_HEIGHT = 0;
  public static double CLIMB_MAX = 660000;
  public static double OV_CLIMB = 0.1;
  public static double AUTO_LEVEL_PRESET = 12.05 * 4096;

  public static final int kSlotIdx = 0;
  public static final int kPIDLoopIdx = 0;
  public static final int kTimeoutMs = 30;
  public static boolean kSensorPhase = false;
  public static boolean kMotorInvert = false;
  public static final double DEADBAND = 0.05;

  ////// Climb 1 Constants ///////////////////////////
  public static final int kclimb1SlotIdx = 0;
  public static final int kclimb1PIDLoopIdx = 0;
  public static final int kclimb1TimeoutMs = 30;
  public static boolean kclimb1SensorPhase = true;
  public static boolean kclimb1MotorInvert = false;
  // public static final Gains kGains = new Gains(0.5, 0.0, 0.1, 0.00, 0, 0.5);
  public static final Gains kGainsR1 = new Gains(0.5, 0.0, 0.5, 0.0, 0, 1.0);

  ////// Climb 2 Constants ///////////////////////////
  public static final int kclimb2SlotIdx = 0;
  public static final int kclimb2PIDLoopIdx = 0;
  public static final int kclimb2TimeoutMs = 30;
  public static boolean kclimb2SensorPhase = true;
  public static boolean kclimb2MotorInvert = false;
  public static final Gains kGainsR2 =
      new Gains(0.5, 0.0, 0.5, 0.0, 0, 1.0); // Move Ball to shooter

  // Shelf
  public static int PneumaticsShelf = 0;

  // Shooter

  public static int PN_SHELF = 0;
  public static int ZONE_LOW2 = 250 * 42; // not used
  public static int ZONE_LOW = 400 * 42; // not used
  public static int ZONE_HIGH = 2500 * 42 * 100; // not used
  // public static double ZONE_LOW_VC2 = 0.38; //  no shelf
  // public static double ZONE_LOW_VC = 0.36; // with shelf
  //  public static double ZONE_HIGH_VC = 1.0;
  public static double ZONE_LOW_VC2 = 0.2; //  no shelf
  public static double ZONE_LOW_VC = 0.2; // with shelf
  public static double ZONE_HIGH_VC = 0.5;
  public static double ZONE_HIGH_VC2 = 1.0;
  public static double kR0 = 250;
  public static double kR1 = 1500;

  public static double kshootRightP = 0.01; // 6e-5
  public static double kshootRightI = 0;
  public static double kshootRightD = 0;
  public static double kshootRightIz = 0;
  public static double kshootRightF = 0.000015;
  public static double kshootRightkMaxOutput = 0.2;
  public static double kshootRightkMinOutput = -0.2;
  public static double kshootRightmaxRPM = 1500;

  public static double kshootLeftP = 0.01;
  public static double kshootLeftI = 0;
  public static double kshootLeftD = 0;
  public static double kshootLeftIz = 0;
  public static double kshootLeftF = 0.000015;
  public static double kshootLeftkMaxOutput = 0.2;
  public static double kshootLeftkMinOutput = -0.2;
  public static double kshootLeftmaxRPM = 1500;

  ///////////////// CAN Constants //////////////////////////////

  public static int PH_CAN = 1;
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
}
