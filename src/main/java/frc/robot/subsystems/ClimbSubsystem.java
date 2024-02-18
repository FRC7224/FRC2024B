package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.GlobalStatus;

@SuppressWarnings("removal")

/** */
public class ClimbSubsystem extends SubsystemBase {

  private static WPI_TalonFX climb1 = new WPI_TalonFX(Constants.CLIMB_PORT_L);
  private static WPI_TalonFX climb2 = new WPI_TalonFX(Constants.CLIMB_PORT_R);

  /** Used to create string thoughout loop */
  StringBuilder _sb = new StringBuilder();

  int _loops = 0;

  /** Save the target position to servo to */
  double targetPositionRotations;

  public ClimbSubsystem() {

    ///////////////// Motor One ////////////////////////////////////////////////
    /* Factory Default all hardware to prevent unexpected behaviour */
    climb1.configFactoryDefault();

    /* Config the sensor used for Primary PID and sensor direction */
    climb1.configSelectedFeedbackSensor(
        TalonFXFeedbackDevice.IntegratedSensor,
        Constants.kclimb1PIDLoopIdx,
        Constants.kclimb1TimeoutMs);

    /* Ensure sensor is positive when output is positive */
    climb1.setSensorPhase(Constants.kclimb1SensorPhase);

    /**
     * Set based on what direction you want forward/positive to be. This does not affect sensor
     * phase.
     */
    climb1.setInverted(Constants.kclimb1MotorInvert);
    /*
     * Talon FX does not need sensor phase set for its integrated sensor
     * This is because it will always be correct if the selected feedback device is
     * integrated sensor (default value)
     * and the user calls getSelectedSensor* to get the sensor's position/velocity.
     *
     * https://phoenix-documentation.readthedocs.io/en/latest/ch14_MCSensor.html#
     * sensor-phase
     */
    // climb1.setSensorPhase(true);

    /* Config the peak and nominal outputs, 12V means full */
    climb1.configNominalOutputForward(0, Constants.kclimb1TimeoutMs);
    climb1.configNominalOutputReverse(0, Constants.kclimb1TimeoutMs);
    climb1.configPeakOutputForward(1, Constants.kclimb1TimeoutMs);
    climb1.configPeakOutputReverse(-1, Constants.kclimb1TimeoutMs);

    /**
     * Config the allowable closed-loop error, Closed-Loop output will be neutral within this range.
     * See Table in Section 17.2.1 for native units per rotation.
     */
    climb1.configAllowableClosedloopError(
        0, Constants.kclimb1PIDLoopIdx, Constants.kclimb1TimeoutMs);

    /* Config Position Closed Loop gains in slot0, tsypically kF stays zero. */
    climb1.config_kF(Constants.kPIDLoopIdx, Constants.kGainsR1.kF, Constants.kclimb1TimeoutMs);
    climb1.config_kP(Constants.kPIDLoopIdx, Constants.kGainsR1.kP, Constants.kclimb1TimeoutMs);
    climb1.config_kI(Constants.kPIDLoopIdx, Constants.kGainsR1.kI, Constants.kclimb1TimeoutMs);
    climb1.config_kD(Constants.kPIDLoopIdx, Constants.kGainsR1.kD, Constants.kclimb1TimeoutMs);

    ///////////////// Motor Two ////////////////////////////////////////////////
    /* Factory Default all hardware to prevent unexpected behaviour */
    climb2.configFactoryDefault();

    /* Config the sensor used for Primary PID and sensor direction */
    climb2.configSelectedFeedbackSensor(
        TalonFXFeedbackDevice.IntegratedSensor,
        Constants.kclimb2PIDLoopIdx,
        Constants.kclimb2TimeoutMs);

    /* Ensure sensor is positive when output is positive */
    climb2.setSensorPhase(Constants.kclimb2SensorPhase);

    /**
     * Set based on what direction you want forward/positive to be. This does not affect sensor
     * phase.
     */
    climb2.setInverted(Constants.kclimb2MotorInvert);
    /*
     * Talon FX does not need sensor phase set for its integrated sensor
     * This is because it will always be correct if the selected feedback device is
     * integrated sensor (default value)
     * and the user calls getSelectedSensor* to get the sensor's position/velocity.
     *
     * https://phoenix-documentation.readthedocs.io/en/latest/ch14_MCSensor.html#
     * sensor-phase
     */
    // climb1.setSensorPhase(true);

    /* Config the peak and nominal outputs, 12V means full */
    climb2.configNominalOutputForward(0, Constants.kclimb2TimeoutMs);
    climb2.configNominalOutputReverse(0, Constants.kclimb2TimeoutMs);
    climb2.configPeakOutputForward(1, Constants.kclimb2TimeoutMs);
    climb2.configPeakOutputReverse(-1, Constants.kclimb2TimeoutMs);

    /**
     * Config the allowable closed-loop error, Closed-Loop output will be neutral within this range.
     * See Table in Section 17.2.1 for native units per rotation.
     */
    climb2.configAllowableClosedloopError(
        0, Constants.kclimb1PIDLoopIdx, Constants.kclimb1TimeoutMs);

    /* Config Position Closed Loop gains in slot0, tsypically kF stays zero. */
    climb2.config_kF(Constants.kPIDLoopIdx, Constants.kGainsR2.kF, Constants.kclimb2TimeoutMs);
    climb2.config_kP(Constants.kPIDLoopIdx, Constants.kGainsR2.kP, Constants.kclimb2TimeoutMs);
    climb2.config_kI(Constants.kPIDLoopIdx, Constants.kGainsR2.kI, Constants.kclimb2TimeoutMs);
    climb2.config_kD(Constants.kPIDLoopIdx, Constants.kGainsR2.kD, Constants.kclimb2TimeoutMs);
  }

  /// Motor 1 Methods ////
  public static double GetMotorOutputPercentR1() {
    return (climb1.getMotorOutputPercent());
  }
  ;

  public static double GetSelectedSensorPositionR1() {
    return (climb1.getSelectedSensorPosition(0));
  }
  ;

  public double GetClosedLoopErrorR1() {
    return (climb1.getClosedLoopError(0));
  }
  ;

  public ControlMode GetControlModeR1() {
    return (climb1.getControlMode());
  }
  ;

  public void SetTargetPositionClimb1(double targetPositionRotations) {
    climb1.set(ControlMode.Position, targetPositionRotations);
    GlobalStatus.Global_Climb1_position = targetPositionRotations;
  }
  ;

  public void SetPercentOutputR1(double percentoutput) {
    climb1.set(ControlMode.PercentOutput, percentoutput);
  }
  ;

  /// Motor 2 Methods ////

  public static double GetMotorOutputPercentR2() {
    return (climb2.getMotorOutputPercent());
  }
  ;

  public static double GetSelectedSensorPositionR2() {
    return (climb2.getSelectedSensorPosition(0));
  }
  ;

  public double GetClosedLoopErrorR2() {
    return (climb2.getClosedLoopError(0));
  }
  ;

  public ControlMode GetControlModeR2() {
    return (climb2.getControlMode());
  }
  ;

  public void SetTargetPositionClimb2(double targetPositionRotations) {
    climb2.set(ControlMode.Position, targetPositionRotations);
    GlobalStatus.Global_Climb2_position = targetPositionRotations;
  }
  ;

  public void SetPercentOutputR2(double percentoutput) {
    climb2.set(ControlMode.PercentOutput, percentoutput);
  }
  ;

  // public void SetTargetRotStart() {
  // climb1.set(ControlMode.Position, Constants.START_ROT_PRESET);
  // climb2.set(ControlMode.Position, -Constants.START_ROT_PRESET);
  // GlobalStatus.Global_Climb1_position = Constants.START_ROT_PRESET;
  // GlobalStatus.Global_Climb2_position = -Constants.START_ROT_PRESET;
  // }
  // ;

  // public void SetTargetRotHIGH() {
  // climb1.set(ControlMode.Position, Constants.HIGH_ROT_PRESET +
  // Constants.OFFSET_ROT);
  // climb2.set(ControlMode.Position, -Constants.HIGH_ROT_PRESET);
  // GlobalStatus.Global_Climb1_position = Constants.HIGH_ROT_PRESET +
  // Constants.OFFSET_ROT;
  // GlobalStatus.Global_Climb2_position = -Constants.HIGH_ROT_PRESET;
  // }
  // ;

  // public void SetTargetRotZero() {
  // climb1.set(ControlMode.Position, 0);
  // climb2.set(ControlMode.Position, 0);
  // GlobalStatus.Global_Climb1_position = 0;
  // GlobalStatus.Global_Climb2_position = 0;
  // }

  /** Stops the motion of the robot. */
  public void stop() {
    climb1.set(ControlMode.PercentOutput, 0);
    climb2.set(ControlMode.PercentOutput, 0);
  }
  ;

  @Override
  public void periodic() {

    // This method will be called once per scheduler run

  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run when in simulation

  }

  // Put methods for controlling this subsystem
  // here. Call these from Commands.

}
