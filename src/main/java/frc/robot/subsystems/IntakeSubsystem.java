package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

@SuppressWarnings("removal")

/** */
public class IntakeSubsystem extends SubsystemBase {

  private static WPI_TalonSRX intake = new WPI_TalonSRX(Constants.INTAKE_PORT);
  private static CANSparkMax elevatorleft =
      new CANSparkMax(Constants.ELEVATOR_L, MotorType.kBrushless);
  private static CANSparkMax elevatorright =
      new CANSparkMax(Constants.ELEVATOR_R, MotorType.kBrushless);

  // private final Timer timer = new Timer();

  public IntakeSubsystem() {}

  /** Set the intake speed */
  public void SetIntakeOn() {
    intake.set(Constants.INTAKE_SPEED);
  }

  /** Set Intake off */
  public void SetIntakeOff() {
    intake.set(0);
  }

  /** Set the elevator speed */
  public void SetElevatorOn() {
    elevatorleft.set(Constants.ELEVATOR_SPEED);
    elevatorright.set(Constants.ELEVATOR_SPEED);
  }

  /** Set elevator off */
  public void SetElevatorOff() {
    elevatorleft.set(0);
    elevatorright.set(0);
  }

  /**
   * Squares the specified value, while preserving the sign. This method is used on all joystick
   * inputs. This is useful as a non-linear range is more natural for the driver.
   *
   * @param value
   * @return
   */
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
