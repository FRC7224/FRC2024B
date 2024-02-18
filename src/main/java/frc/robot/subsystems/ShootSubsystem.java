package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

/** */
public class ShootSubsystem extends SubsystemBase {

  private WPI_TalonFX shootMotorRight = new WPI_TalonFX(Constants.SHOOT_PORT_L, "rio");
  private WPI_TalonFX shootMotorLeft = new WPI_TalonFX(Constants.SHOOT_PORT_R, "rio");

  //    private WPI_TalonFX shootMotorRight= new WPI_TalonFX(Constants.kShootMotorLeftPort,"rio");
  //   private WPI_TalonFX shootMotorLeft = new WPI_TalonFX(Constants.kShootMotorRightPort,"rio");

  Zone[] zones = {
    new Zone(Constants.kT0, Constants.kB0),
    new Zone(Constants.kT1, Constants.kB1),
    new Zone(Constants.kT2, Constants.kB2),
    new Zone(Constants.kT3, Constants.kB3),
    new Zone(Constants.kT4, Constants.kB4),
    new Zone(Constants.kT5, Constants.kB5),
    new Zone(Constants.kT6, Constants.kB6),
    new Zone(Constants.kT7, Constants.kB7),
    new Zone(Constants.kT8, Constants.kB8),
    new Zone(Constants.kT9, Constants.kB9),
    new Zone(Constants.kT10, Constants.kB10),
    new Zone(Constants.kT11, Constants.kB11),
    new Zone(Constants.kT12, Constants.kB12),
    new Zone(Constants.kT13, Constants.kB13),
    new Zone(Constants.kT14, Constants.kB14),
    new Zone(Constants.kT15, Constants.kB15),
    new Zone(Constants.kT16, Constants.kB16),
    new Zone(Constants.kT17, Constants.kB17),
    new Zone(Constants.kT18, Constants.kB18),
    new Zone(Constants.kT19, Constants.kB19),
    new Zone(Constants.kT20, Constants.kB20),
    new Zone(Constants.kT21, Constants.kB21),
    new Zone(Constants.kT22, Constants.kB22),
    new Zone(Constants.kT23, Constants.kB23),
    new Zone(Constants.kT24, Constants.kB24),
    new Zone(Constants.kT25, Constants.kB25),
    ///  special short zone
    new Zone(Constants.kT26, Constants.kB26)
  };

  /** */
  public ShootSubsystem() {
    /*
     * sets up shooter with PID
     */

    shootMotorRight.configFactoryDefault();

    shootMotorRight.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
    shootMotorRight.set(ControlMode.Velocity, 0);
    shootMotorRight.setInverted(false);
    shootMotorRight.setSensorPhase(true);

    /* Config the peak and nominal outputs */
    shootMotorRight.configNominalOutputForward(0, Constants.kTimeoutMs);
    shootMotorRight.configNominalOutputReverse(0, Constants.kTimeoutMs);
    shootMotorRight.configPeakOutputForward(100, Constants.kTimeoutMs);
    shootMotorRight.configPeakOutputReverse(9, Constants.kTimeoutMs);

    /* Config the Velocity closed loop gains in slot0 */
    shootMotorRight.config_kF(Constants.kPIDLoopIdx, Constants.kshootRightF, Constants.kTimeoutMs);
    shootMotorRight.config_kP(Constants.kPIDLoopIdx, Constants.kshootRightP, Constants.kTimeoutMs);
    shootMotorRight.config_kI(Constants.kPIDLoopIdx, Constants.kshootRightI, Constants.kTimeoutMs);
    shootMotorRight.config_kD(Constants.kPIDLoopIdx, Constants.kshootRightD, Constants.kTimeoutMs);

    shootMotorLeft.configFactoryDefault();
    shootMotorLeft.set(ControlMode.Velocity, 0);
    shootMotorLeft.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
    shootMotorLeft.setSensorPhase(true);
    shootMotorLeft.setInverted(false);

    /* Config the peak and nominal outputs */
    shootMotorLeft.configNominalOutputForward(0, Constants.kTimeoutMs);
    shootMotorLeft.configNominalOutputReverse(0, Constants.kTimeoutMs);
    shootMotorLeft.configPeakOutputForward(100, Constants.kTimeoutMs);
    shootMotorLeft.configPeakOutputReverse(0, Constants.kTimeoutMs);

    /* Config the Velocity closed loop gains in slot0 */
    shootMotorLeft.config_kF(Constants.kPIDLoopIdx, Constants.kshootLeftF, Constants.kTimeoutMs);
    shootMotorLeft.config_kP(Constants.kPIDLoopIdx, Constants.kshootLeftP, Constants.kTimeoutMs);
    shootMotorLeft.config_kI(Constants.kPIDLoopIdx, Constants.kshootLeftI, Constants.kTimeoutMs);
    shootMotorLeft.config_kD(Constants.kPIDLoopIdx, Constants.kshootLeftD, Constants.kTimeoutMs);
  }

  /** sets the shooter speed */
  public void setShootSpeed(int zoneposition) {
    // shootMotor1.set(speed);
    SmartDashboard.putNumber("top", zones[zoneposition].getTopMotor());
    SmartDashboard.putNumber("zoneposition", zoneposition);
    shootMotorRight.set(ControlMode.Velocity, zones[zoneposition].getTopMotor());
    shootMotorLeft.set(ControlMode.Velocity, zones[zoneposition].getBottomMotor());
  }

  /** stop the shooter speed */
  public void stopshooter() {
    shootMotorRight.set(ControlMode.Velocity, 0);
    shootMotorLeft.set(ControlMode.Velocity, 0);
  }

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
