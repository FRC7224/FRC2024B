package frc.robot.commands;

import com.ctre.phoenix.motorcontrol.ControlMode;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants;
import frc.robot.GlobalStatus;
import frc.robot.subsystems.IntakeSubsystem;
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
public class IntakeCommand extends Command {

  private final IntakeSubsystem intakesubsystem;
  private final JoystickButton intakeoverrideButton;
  private final JoystickButton intakeButton;



  
  public IntakeCommand(
      IntakeSubsystem intakesubsystem,
      JoystickButton intakeoverrideButton,
      JoystickButton intakeButton) {
    this.intakesubsystem = intakesubsystem;
    this.intakeoverrideButton = intakeoverrideButton;
    this.intakeButton = intakeButton;
;

    addRequirements(intakesubsystem);
  }

  public void execute() {
    /* Gamepad processing */

    /* Get Talon/Victor's current output percentage */
    /**

     */
    if (intakeoverrideButton.getAsBoolean()) {
      /* When button is held, reverse  */
      /* Percent Output */
      intakesubsystem.SetPercentOutput(extendcontrol * Constants.OV_ARM);
    } else if (drvgoalButton.getAsBoolean()) {
      targetPositionRotations = Constants.DRV_ARM_PRESET;
      intakesubsystem.SetTargetPositionRotations(targetPositionRotations);
    } else if (medgoalButton.getAsBoolean()) {
      targetPositionRotations = Constants.MED_ARM_PRESET;
      intakesubsystem.SetTargetPositionRotations(targetPositionRotations);
    } else if (highgoalButton.getAsBoolean()) {
      targetPositionRotations = Constants.HIGH_ARM_PRESET;
      intakesubsystem.SetTargetPositionRotations(targetPositionRotations);
    } else {
      /* Position Closed Loop */
      /* 7.5 Rotations * 4096 u/rev in either direction */
      extendamount = extendcontrol;
      if (extendcontrol <= 0.0) {
        extendamount = 0;
      }
      targetPositionRotations = (extendamount * Constants.MAX_ARM);
      if ((Math.abs(GlobalStatus.Global_Rotate1_position) <= Constants.LOW48_ROT)
          && (targetPositionRotations >= Constants.MAX48_ARM)) {
        targetPositionRotations = Constants.MAX48_ARM;
      }
      ;

      if ((Math.abs(GlobalStatus.Global_Rotate1_position) >= Constants.HIGH48_ROT)
          && (targetPositionRotations >= Constants.MAX48_ARM)) {
        targetPositionRotations = Constants.MAX48_ARM;
      }
      ;

      intakesubsystem.SetTargetPositionRotations(targetPositionRotations - Constants.ARM_OFFSET);
    }

    /* If Talon is in position closed-loop, print some more info */
    if (intakesubsystem.GetControlMode() == ControlMode.Position) {
      /* ppend more signals to print when in speed mode. */
      //   _sb.append("\terr:");
      //    _sb.append(intakesubsystem.GetClosedLoopError());
      //   _sb.append("u"); // Native Units

      //   _sb.append("\ttrg:");
      //   _sb.append(targetPositionRotations);
      //   _sb.append("u"); // / Native Units
    }

    /** Print every ten loops, printing too much too fast is generally bad for performance. */
    if (++_loops >= 10) {
      _loops = 0;
      //   System.out.println(_sb.toString());
    }

    /* Reset built string for next loop */
    _sb.setLength(0);
  }

  @Override
  public void end(boolean interrupted) {
    this.intakesubsystem.stop();

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
