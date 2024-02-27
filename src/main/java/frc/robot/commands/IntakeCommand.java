package frc.robot.commands;

// import com.google.flatbuffers.Constants;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants;
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
  private boolean InTakeInProgress = false;
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
    boolean ballloaded;
    ballloaded = !intakesubsystem.GetNoteLoadStatus();

    if (intakeButton.getAsBoolean()) {
      timer.start();
      timer.reset();
    }
    ;
    SmartDashboard.putBoolean("intake button", intakeButton.getAsBoolean());
    SmartDashboard.putBoolean("intake override", intakeoverrideButton.getAsBoolean());
    if (intakeoverrideButton.getAsBoolean()) {
      /* When button is held override */
      /* Percent Output */
      intakesubsystem.SetIntakeOn();
      intakesubsystem.SetElevatorOn();
    } else if (intakeButton.getAsBoolean() || (InTakeInProgress)) {
      InTakeInProgress = true;
      if (timer.get() <= Constants.INTAKE_TIMER) {

        SmartDashboard.putNumber("intake timer inside", timer.get());
        if (ballloaded == false) { //  ball bot loaded and time active
          intakesubsystem.SetIntakeOn();
          intakesubsystem.SetElevatorOn();
        } else { // Ball loaded stop and reset
          InTakeInProgress = false;
          intakesubsystem.SetIntakeOff();
          intakesubsystem.SetElevatorOff();
          timer.reset();
          timer.stop();
        }
      } else { // timer expired clean up and reset
        InTakeInProgress = false;
        intakesubsystem.SetIntakeOff();
        intakesubsystem.SetElevatorOff();
        timer.reset();
        timer.stop();
      }
    } else { // cleanup after override botton
      InTakeInProgress = false;
      intakesubsystem.SetIntakeOff();
      intakesubsystem.SetElevatorOff();
      timer.reset();
      timer.stop();
    }
  }
}
