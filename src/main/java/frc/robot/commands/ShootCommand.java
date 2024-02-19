package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants;
import frc.robot.subsystems.ShootSubsystem;
import java.util.function.DoubleSupplier;

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
public class ShootCommand extends Command {

  private final ShootSubsystem shootsubsystem;
  private final JoystickButton shootButton;
  private final DoubleSupplier translationXSupplier;
  int shootpower = 1;

  public ShootCommand(
      ShootSubsystem shootsubsystem,
      JoystickButton shootButton,
      DoubleSupplier translationXSupplier) {
    this.shootsubsystem = shootsubsystem;
    this.shootButton = shootButton;
    this.translationXSupplier = translationXSupplier;

    addRequirements(shootsubsystem);
  }

  public void execute() {

    double shootdistance = modifyAxis(translationXSupplier.getAsDouble());

    if (shootButton.getAsBoolean()) {
      if (shootdistance <= Constants.SHOOT_BOTTOM) {
        shootpower = 0;
      } else if (shootdistance <= Constants.SHOOT_MIDDLE) {
        shootpower = 1;
      } else {
        shootpower = 2;
      }
      shootsubsystem.setShootSpeed(shootpower);
    } else {
      shootsubsystem.stopshooter();
    }
  }

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
