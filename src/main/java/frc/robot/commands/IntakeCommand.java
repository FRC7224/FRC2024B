package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
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
  private final Timer timer = new Timer();

  public IntakeCommand(
      IntakeSubsystem intakesubsystem,
      JoystickButton intakeoverrideButton,
      JoystickButton intakeButton) {
    this.intakesubsystem = intakesubsystem;
    this.intakeoverrideButton = intakeoverrideButton;
    this.intakeButton = intakeButton;
    addRequirements(intakesubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer.reset();
  }

  public void execute() {

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

  /*
    if (launchInProgress) {
      if (timer.get() <= Constants.kshooterTimer_spin) {
          System.out.print("spinup");
          m_shootsubsystem.setShootSpeed(zonePosition);

      } else if (timer.get() <= Constants.kshooterTimer_timer) {
          System.out.print("shoot ing");
          m_shootsubsystem.setShootSpeed(zonePosition);
          m_shootsubsystem.setelvSpeed(Constants.kelvspeed);
          ballshot = true;
          if (timer.get() <= (Constants.kshooterTimer_timer+ 0.75)){  // 0.5 second before pushing
              m_shootsubsystem.pushBall();
          }
      } else {
          launchInProgress = false;
          timer.reset();
          timer.stop();
      }
  } else {
      m_shootsubsystem.stopshooter();
      m_shootsubsystem.setelvSpeed(0);
      m_shootsubsystem.resetBallPush();
      if (ballshot) { // first time after a ball has been shot
      //    new SequentialCommandGroup(new MoveBalltoShooterTimed(m_intakesubsystem));
          Constants.LAUNCHREADY = false;
          ballshot = false;
          launchReady = false;
      }

  */

}
