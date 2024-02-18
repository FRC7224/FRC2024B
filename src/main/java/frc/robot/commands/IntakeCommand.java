package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.subsystems.IntakeSubsystem;

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
    /** */
    if (intakeoverrideButton.getAsBoolean()) {
      /* When button is held override */
      /* Percent Output */
      intakesubsystem.SetIntakeOn();
      intakesubsystem.SetElevatorOn();
    } else if (intakeButton.getAsBoolean()) {
      intakesubsystem.SetIntakeOn();
      intakesubsystem.SetElevatorOn();
    }
    intakesubsystem.SetIntakeOff();
    intakesubsystem.SetElevatorOff();
  }
}
