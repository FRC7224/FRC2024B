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

  Zone[] zones = {
    new Zone(Constants.kL0, Constants.kR0),
    new Zone(Constants.kL1, Constants.kR1),
    new Zone(Constants.kL2, Constants.kR2),
  };

  /** */
  public ShootSubsystem() {
    /*
     * sets up RIGHT shooter with PID
     */
    shootMotorRight.configFactoryDefault();
    shootMotorRight.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
    shootMotorRight.set(ControlMode.Velocity, 0);
    shootMotorRight.setInverted(false);
    shootMotorRight.setSensorPhase(true);

    /*  RIGHT Config the peak and nominal outputs */
    shootMotorRight.configNominalOutputForward(0, Constants.kTimeoutMs);
    shootMotorRight.configNominalOutputReverse(0, Constants.kTimeoutMs);
    shootMotorRight.configPeakOutputForward(100, Constants.kTimeoutMs);
    shootMotorRight.configPeakOutputReverse(9, Constants.kTimeoutMs);

    /* RIGHT Config the Velocity closed loop gains in slot0 */
    shootMotorRight.config_kF(Constants.kPIDLoopIdx, Constants.kshootRightF, Constants.kTimeoutMs);
    shootMotorRight.config_kP(Constants.kPIDLoopIdx, Constants.kshootRightP, Constants.kTimeoutMs);
    shootMotorRight.config_kI(Constants.kPIDLoopIdx, Constants.kshootRightI, Constants.kTimeoutMs);
    shootMotorRight.config_kD(Constants.kPIDLoopIdx, Constants.kshootRightD, Constants.kTimeoutMs);

    /*
     * sets up LEFT shooter with PID
     */
    shootMotorLeft.configFactoryDefault();
    shootMotorLeft.set(ControlMode.Velocity, 0);
    shootMotorLeft.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
    shootMotorLeft.setSensorPhase(true);
    shootMotorLeft.setInverted(false);

    /* LEFT Config the peak and nominal outputs */
    shootMotorLeft.configNominalOutputForward(0, Constants.kTimeoutMs);
    shootMotorLeft.configNominalOutputReverse(0, Constants.kTimeoutMs);
    shootMotorLeft.configPeakOutputForward(100, Constants.kTimeoutMs);
    shootMotorLeft.configPeakOutputReverse(0, Constants.kTimeoutMs);

    /* LEFT Config the Velocity closed loop gains in slot0 */
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
