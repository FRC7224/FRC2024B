package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

@SuppressWarnings("removal")

/** */
public class ShootSubsystem extends SubsystemBase {

  private CANSparkMax shootMotorRight =
      new CANSparkMax(
          Constants.SHOOT_PORT_R, com.revrobotics.CANSparkLowLevel.MotorType.kBrushless);
  private CANSparkMax shootMotorLeft =
      new CANSparkMax(
          Constants.SHOOT_PORT_L, com.revrobotics.CANSparkLowLevel.MotorType.kBrushless);
  private SparkPIDController m_pidControllerRight;
  private SparkPIDController m_pidControllerLeft;
  private RelativeEncoder m_encoderRight;
  private RelativeEncoder m_encoderLeft;
  /** */
  public ShootSubsystem() {
    /*
     * sets up RIGHT shooter with PID
     */
    /**
     * The RestoreFactoryDefaults method can be used to reset the configuration parameters in the
     * SPARK MAX to their factory default state. If no argument is passed, these parameters will not
     * persist between power cycles
     */
    shootMotorRight.restoreFactoryDefaults();

    /**
     * In order to use PID functionality for a controller, a SparkPIDController object is
     * constructed by calling the getPIDController() method on an existing CANSparkMax object
     */
    m_pidControllerRight = shootMotorRight.getPIDController();

    // Encoder object created to display position values
    m_encoderRight = shootMotorRight.getEncoder();

    // set PID coefficients
    m_pidControllerRight.setP(Constants.kshootRightP);
    m_pidControllerRight.setI(Constants.kshootRightI);
    m_pidControllerRight.setD(Constants.kshootRightD);
    m_pidControllerRight.setIZone(Constants.kshootRightIz);
    m_pidControllerRight.setFF(Constants.kshootRightF);
    m_pidControllerRight.setOutputRange(
        Constants.kshootRightkMinOutput, Constants.kshootRightkMinOutput);

    // display PID coefficients on SmartDashboard
    SmartDashboard.putNumber("R P Gain", Constants.kshootRightP);
    SmartDashboard.putNumber("R I Gain", Constants.kshootRightI);
    SmartDashboard.putNumber("R D Gain", Constants.kshootRightD);
    SmartDashboard.putNumber("R I Zone", Constants.kshootRightIz);
    SmartDashboard.putNumber("R Feed Forward", Constants.kshootRightF);
    SmartDashboard.putNumber("R Max Output", Constants.kshootRightkMinOutput);
    SmartDashboard.putNumber("R Min Output", Constants.kshootRightkMinOutput);

    /*
     * sets up LEFT shooter with PID
     */
    /**
     * The RestoreFactoryDefaults method can be used to reset the configuration parameters in the
     * SPARK MAX to their factory default state. If no argument is passed, these parameters will not
     * persist between power cycles
     */
    shootMotorLeft.restoreFactoryDefaults();

    /**
     * In order to use PID functionality for a controller, a SparkPIDController object is
     * constructed by calling the getPIDController() method on an existing CANSparkMax object
     */
    m_pidControllerLeft = shootMotorLeft.getPIDController();

    // Encoder object created to display position values
    m_encoderLeft = shootMotorLeft.getEncoder();

    // set PID coefficients
    m_pidControllerLeft.setP(Constants.kshootLeftP);
    m_pidControllerLeft.setI(Constants.kshootLeftI);
    m_pidControllerLeft.setD(Constants.kshootLeftD);
    m_pidControllerLeft.setIZone(Constants.kshootLeftIz);
    m_pidControllerLeft.setFF(Constants.kshootLeftF);
    m_pidControllerLeft.setOutputRange(
        Constants.kshootLeftkMinOutput, Constants.kshootLeftkMinOutput);

    // display PID coefficients on SmartDashboard
    SmartDashboard.putNumber("R P Gain", Constants.kshootLeftP);
    SmartDashboard.putNumber("R I Gain", Constants.kshootLeftI);
    SmartDashboard.putNumber("R D Gain", Constants.kshootLeftD);
    SmartDashboard.putNumber("R I Zone", Constants.kshootLeftIz);
    SmartDashboard.putNumber("R Feed Forward", Constants.kshootLeftF);
    SmartDashboard.putNumber("R Max Output", Constants.kshootLeftkMinOutput);
    SmartDashboard.putNumber("R Min Output", Constants.kshootLeftkMinOutput);
  }

  /** sets the shooter speed */
  public void setShootSpeedLow() {
    shootMotorRight.set(-.4);
    shootMotorLeft.set(.4);
    SmartDashboard.putNumber("Right SHoot Speed", m_encoderRight.getVelocity());

    //  m_pidControllerRight.setReference(Constants.ZONE_LOW, CANSparkMax.ControlType.kVelocity);
    SmartDashboard.putNumber("Left SHoot Speed", m_encoderLeft.getVelocity());
    //  m_pidControllerLeft.setReference(Constants.ZONE_LOW, CANSparkMax.ControlType.kVelocity);
  }

  public void setShootSpeedHigh() {

    shootMotorRight.set(-.9);
    shootMotorLeft.set(.9);
    SmartDashboard.putNumber("Right SHoot Speed", m_encoderRight.getVelocity());
    //    m_pidControllerRight.setReference(Constants.ZONE_HIGH, CANSparkMax.ControlType.kVelocity);
    SmartDashboard.putNumber("Left SHoot Speed", m_encoderLeft.getVelocity());
    //    m_pidControllerLeft.setReference(Constants.ZONE_HIGH, CANSparkMax.ControlType.kVelocity);
  }

  public void stopshooter() {
    SmartDashboard.putNumber("Right SHoot Speed", m_encoderRight.getVelocity());
    SmartDashboard.putNumber("Left SHoot Speed", m_encoderLeft.getVelocity());
    m_pidControllerLeft.setReference(0, CANSparkMax.ControlType.kVelocity);
    m_pidControllerRight.setReference(0, CANSparkMax.ControlType.kVelocity);
    shootMotorRight.stopMotor();
    shootMotorLeft.stopMotor();
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
