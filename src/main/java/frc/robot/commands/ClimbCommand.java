package frc.robot.commands;

import com.ctre.phoenix.motorcontrol.ControlMode;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants;
import frc.robot.subsystems.ClimbSubsystem;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;

/**
 * This command, when executed, instructs the drivetrain subsystem to drive based on the specified
 * values from the controller(s). This command is designed to be the default command for the
 * drivetrain subsystem.
 *
 * <p>Requires: the Drivetrain subsystem
 *
 * <p>Finished When: never
 *
 * <p>At End: stops the drivetrain
 */
public class ClimbCommand extends Command {

  private final ClimbSubsystem climbsubsystem;
  private final DoubleSupplier translationRSupplier;
  private final JoystickButton climboverrideButton;
  private final JoystickButton autolevelButton;

  double targetPositionRotations = 0;
  /** Used to create string thoughout loop */
  StringBuilder _sb = new StringBuilder();

  int _loops = 0;

  /**
   * Create a new ArmExtendCommand command object.
   *
   * @param armsubsystem the drivetrain subsystem instructed by this command
   * @param translationXSupplier the supplier of the translation x value as a percentage of the
   *     maximum velocity as defined by the standard field or robot coordinate system
   * @return
   */
  public ClimbCommand(
      ClimbSubsystem climbsubsystem,
      JoystickButton climboverrideButton,
      JoystickButton autoleveButton,
      DoubleSupplier translationRSupplier) {
    this.climbsubsystem = climbsubsystem;
    this.climboverrideButton = climboverrideButton;
    this.autolevelButton = autoleveButton;
    this.translationRSupplier = translationRSupplier;

    addRequirements(climbsubsystem);
  }

  public void execute() {
    /* Gamepad processing */

    double climbcontrol = modifyAxis(translationRSupplier.getAsDouble());

    double motorOutput;

    /* Get Talon/Victor's current output percentage */
    motorOutput = ClimbSubsystem.GetMotorOutputPercentR1();

    /* Prepare line to print */
    _sb.append("\tout:");
    /* Cast to int to remove decimal places */
    _sb.append((int) (motorOutput * 100));
    _sb.append("%"); // Percent

    _sb.append("\tpos:");
    _sb.append(ClimbSubsystem.GetSelectedSensorPositionR1());
    _sb.append("u"); // Native units

    /**
     * When button 1 is pressed, perform Position Closed Loop to selected position, indicated by
     * Joystick position x10, [-10, 10] rotations
     */
    if (climboverrideButton.getAsBoolean()) {
      /* When button is held, just straight drive */
      /* Percent Output */
      climbsubsystem.SetPercentOutputR1(climbcontrol * Constants.OV_CLIMB);
      climbsubsystem.SetPercentOutputR2(climbcontrol * Constants.OV_CLIMB);
    } else if (autolevelButton.getAsBoolean()) {
      /// *** NEED TO FINISH *** */
      if (targetPositionRotations >= 0) { // check to see if arm is rotated backwards
        targetPositionRotations = Constants.AUTO_LEVEL_PRESET; // No offest
      } else {
        targetPositionRotations = Constants.AUTO_LEVEL_PRESET;
      }
      climbsubsystem.SetTargetPositionClimb1(targetPositionRotations);
      climbsubsystem.SetTargetPositionClimb2(-targetPositionRotations);
    } else {
      /* Position Closed Loop */
      /* x *  Rotations * 4096 u/rev in either direction */

      if (targetPositionRotations >= 0) { // check to see if arm is rotated backwards
        targetPositionRotations = climbcontrol * Constants.CLIMB_MAX;
      }
      climbsubsystem.SetTargetPositionClimb1(targetPositionRotations);
      climbsubsystem.SetTargetPositionClimb2(targetPositionRotations);
    }

    /* If Talon is in position closed-loop, print some more info */
    if (climbsubsystem.GetControlModeR1() == ControlMode.Position) {
      /* ppend more signals to print when in speed mode. */
      _sb.append("\terr R1:");
      _sb.append(climbsubsystem.GetClosedLoopErrorR1());
      _sb.append("u"); // Native Units

      _sb.append("\terr R2:");
      _sb.append(climbsubsystem.GetClosedLoopErrorR2());
      _sb.append("u"); // Native Units

      _sb.append("\ttrg:");
      _sb.append(targetPositionRotations);
      _sb.append("u"); // / Native Units
    }

    /** Print every ten loops, printing too much too fast is generally bad for performance. */
    if (++_loops >= 10) {
      _loops = 0;
      System.out.println(_sb.toString());
    }

    /* Reset built string for next loop */
    _sb.setLength(0);
  }

  @Override
  public void end(boolean interrupted) {
    this.climbsubsystem.stop();

    super.end(interrupted);

    Logger.recordOutput("ActiveCommands/TeleopSwerve", false);
  }

  /**
   * Squares the specified value, while preserving the sign. This method is used on all joystick
   * inputs. This is useful as a non-linear range is more natural for the driver.
   *
   * @param value
   * @return
   */
  private static double modifyAxis(double value) {
    // Deadband
    value = deadband(value, Constants.DEADBAND);

    // Square the axis
    value = Math.copySign(value * value, value);

    return value;
  }

  private static double deadband(double value, double deadband) {
    if (Math.abs(value) > deadband) {
      if (value > 0.0) {
        return (value - deadband) / (1.0 - deadband);
      } else {
        return (value + deadband) / (1.0 - deadband);
      }
    } else {
      return 0.0;
    }
  }
}
