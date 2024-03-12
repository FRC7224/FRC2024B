// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static frc.robot.subsystems.drivetrain.DrivetrainConstants.*;

import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.lib.team3061.gyro.GyroIO;
import frc.lib.team3061.gyro.GyroIOAhrs;
import frc.lib.team3061.swerve.SwerveModule;
import frc.lib.team3061.swerve.SwerveModuleIO;
import frc.lib.team3061.swerve.SwerveModuleIOSim;
import frc.lib.team3061.swerve.SwerveModuleIOTalonFX;
import frc.robot.Constants.Mode;
import frc.robot.commands.ClimbCommand;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.TeleopSwerve;
import frc.robot.subsystems.ClimbSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShootSubsystem;
import frc.robot.subsystems.drivetrain.Drivetrain;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // private OperatorInterface oi = new OperatorInterface() {};

  final Solenoid shelfextend = new Solenoid(PneumaticsModuleType.REVPH, Constants.PN_SHELF);

  final Joystick drivejoystick = new Joystick(0);
  JoystickButton ShootButton = new JoystickButton(drivejoystick, 1),
      XStanceButton = new JoystickButton(drivejoystick, 2),
      ResetGyroButton = new JoystickButton(drivejoystick, 3),
      FieldRelativeButton = new JoystickButton(drivejoystick, 4),
      ShootButtonLow = new JoystickButton(drivejoystick, 5),
      ShootButtonLow2 = new JoystickButton(drivejoystick, 6),
      Autolevel = new JoystickButton(drivejoystick, 7),
      button8 = new JoystickButton(drivejoystick, 8),
      IntakeButton = new JoystickButton(drivejoystick, 9),
      ClimboverrideButton = new JoystickButton(drivejoystick, 11),
      IntakeoverrideButton = new JoystickButton(drivejoystick, 12);
  // button11 = new JoystickButton(drivejoystick, 11);
  // button12 = new JoystickButton(drivejoystick, 12);

  private Drivetrain drivetrain;
  private IntakeSubsystem intakesubsystem;
  private ClimbSubsystem climbcontrol;
  private ShootSubsystem shootsubsystem;

  // use AdvantageKit's LoggedDashboardChooser instead of SendableChooser to
  // ensure accurate logging
  private final LoggedDashboardChooser<Command> autoChooser =
      new LoggedDashboardChooser<>("Auto Routine");

  // RobotContainer singleton
  private static RobotContainer robotContainer = new RobotContainer();

  /** Create the container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {

    // create real, simulated, or replay subsystems based on the mode and robot
    // specified
    if (Constants.getMode() != Mode.REPLAY) {
      switch (Constants.getRobot()) {
        case ROBOT_2024_SEASON:
          {
            // GyroIO gyro = new Nax X2
            // The important thing about how you configure your gyroscope is that rotating
            // the robot counter-clockwise should
            // cause the angle reading to increase until it wraps back over to zero.
            GyroIO gyro = new GyroIOAhrs(); // NavX connected over MXP SmartDashboard.putNumber

            SwerveModule flModule =
                new SwerveModule(
                    new SwerveModuleIOTalonFX(
                        0,
                        FRONT_LEFT_MODULE_DRIVE_MOTOR,
                        FRONT_LEFT_MODULE_STEER_MOTOR,
                        FRONT_LEFT_MODULE_STEER_ENCODER,
                        FRONT_LEFT_MODULE_STEER_OFFSET),
                    0,
                    MAX_VELOCITY_METERS_PER_SECOND);

            SwerveModule frModule =
                new SwerveModule(
                    new SwerveModuleIOTalonFX(
                        1,
                        FRONT_RIGHT_MODULE_DRIVE_MOTOR,
                        FRONT_RIGHT_MODULE_STEER_MOTOR,
                        FRONT_RIGHT_MODULE_STEER_ENCODER,
                        FRONT_RIGHT_MODULE_STEER_OFFSET),
                    1,
                    MAX_VELOCITY_METERS_PER_SECOND);

            SwerveModule blModule =
                new SwerveModule(
                    new SwerveModuleIOTalonFX(
                        2,
                        BACK_LEFT_MODULE_DRIVE_MOTOR,
                        BACK_LEFT_MODULE_STEER_MOTOR,
                        BACK_LEFT_MODULE_STEER_ENCODER,
                        BACK_LEFT_MODULE_STEER_OFFSET),
                    2,
                    MAX_VELOCITY_METERS_PER_SECOND);

            SwerveModule brModule =
                new SwerveModule(
                    new SwerveModuleIOTalonFX(
                        3,
                        BACK_RIGHT_MODULE_DRIVE_MOTOR,
                        BACK_RIGHT_MODULE_STEER_MOTOR,
                        BACK_RIGHT_MODULE_STEER_ENCODER,
                        BACK_RIGHT_MODULE_STEER_OFFSET),
                    3,
                    MAX_VELOCITY_METERS_PER_SECOND);

            drivetrain = new Drivetrain(gyro, flModule, frModule, blModule, brModule);
            intakesubsystem = new IntakeSubsystem();
            climbcontrol = new ClimbSubsystem();
            shootsubsystem = new ShootSubsystem();
            // new Vision(new VisionIOPhotonVision(CAMERA_NAME));
            break;
          }
        case ROBOT_SIMBOT:
          {
            SwerveModule flModule =
                new SwerveModule(new SwerveModuleIOSim(), 0, MAX_VELOCITY_METERS_PER_SECOND);

            SwerveModule frModule =
                new SwerveModule(new SwerveModuleIOSim(), 1, MAX_VELOCITY_METERS_PER_SECOND);

            SwerveModule blModule =
                new SwerveModule(new SwerveModuleIOSim(), 2, MAX_VELOCITY_METERS_PER_SECOND);

            SwerveModule brModule =
                new SwerveModule(new SwerveModuleIOSim(), 3, MAX_VELOCITY_METERS_PER_SECOND);
            drivetrain = new Drivetrain(new GyroIO() {}, flModule, frModule, blModule, brModule);
            intakesubsystem = new IntakeSubsystem();
            climbcontrol = new ClimbSubsystem();
            shootsubsystem = new ShootSubsystem();

            break;
          }
        default:
          break;
      }

    } else {
      SwerveModule flModule =
          new SwerveModule(new SwerveModuleIO() {}, 0, MAX_VELOCITY_METERS_PER_SECOND);

      SwerveModule frModule =
          new SwerveModule(new SwerveModuleIO() {}, 1, MAX_VELOCITY_METERS_PER_SECOND);

      SwerveModule blModule =
          new SwerveModule(new SwerveModuleIO() {}, 2, MAX_VELOCITY_METERS_PER_SECOND);

      SwerveModule brModule =
          new SwerveModule(new SwerveModuleIO() {}, 3, MAX_VELOCITY_METERS_PER_SECOND);
      drivetrain = new Drivetrain(new GyroIO() {}, flModule, frModule, blModule, brModule);
      intakesubsystem = new IntakeSubsystem();
      climbcontrol = new ClimbSubsystem();
      shootsubsystem = new ShootSubsystem();

      // new Vision(new VisionIO() {});
    }

    // disable all telemetry in the LiveWindow to reduce the processing during each
    // iteration
    LiveWindow.disableAllTelemetry();

    updateOI();

    configureAutoCommands();
  }

  /**
   * This method scans for any changes to the connected joystick. If anything changed, it creates
   * new OI objects and binds all of the buttons to commands.
   */
  public void updateOI() {

    CommandScheduler.getInstance().getActiveButtonLoop().clear();

    /*
     * Set up the default command for the drivetrain. The joysticks' values map to
     * percentage of the
     * maximum velocities. The velocities may be specified from either the robot's
     * frame of
     * reference or the field's frame of reference. In the robot's frame of
     * reference, the positive
     * x direction is forward; the positive y direction, left; position rotation,
     * CCW. In the field
     * frame of reference, the origin of the field to the lower left corner (i.e.,
     * the corner of the
     * field to the driver's right). Zero degrees is away from the driver and
     * increases in the CCW
     * direction. This is why the left joystick's y axis specifies the velocity in
     * the x direction
     * and the left joystick's x axis specifies the velocity in the y direction.
     */
    // drivetrain.setDefaultCommand(
    // new TeleopSwerve(drivetrain,
    // -drivejoystick.getRawAxis(1),-drivejoystick.getRawAxis(0),
    // -drivejoystick.getRawAxis(3));

    drivetrain.setDefaultCommand(
        new TeleopSwerve(
            drivetrain,
            () -> drivejoystick.getRawAxis(1),
            () -> drivejoystick.getRawAxis(0),
            () -> -drivejoystick.getRawAxis(3))); // field vs robot drive

    // pneumaticsSubsystem.setDefaultCommand(new PneumaticsControl(pneumaticsSubsystem));

    intakesubsystem.setDefaultCommand(
        new IntakeCommand( // use same button for preset rotate and extend
            intakesubsystem, IntakeoverrideButton, IntakeButton));

    // shootsubsystem.setDefaultCommand(
    // new ShootCommand( // use same button for preset rotate and extend
    // shootsubsystem, ShootButton, () -> drivejoystick.getRawAxis(4)));

    climbcontrol.setDefaultCommand(
        new ClimbCommand(
            climbcontrol, ClimboverrideButton, Autolevel, () -> -drivejoystick.getRawAxis(2)));

    configureButtonBindings();
  }

  /**
   * Factory method to create the singleton robot container object.
   *
   * @return the singleton robot container object
   */
  public static RobotContainer getInstance() {
    return robotContainer;
  }

  /** Use this method to define your button->command mappings. */
  private void configureButtonBindings() {
    // field-relative toggle

    // SmartDashboard.putBoolean("Field Button input",
    // FieldRelativeButton.getAsBoolean());
    FieldRelativeButton.toggleOnTrue(
        Commands.either(
            Commands.runOnce(drivetrain::disableFieldRelative, drivetrain),
            Commands.runOnce(drivetrain::enableFieldRelative, drivetrain),
            drivetrain::getFieldRelative));

    // Shoot High
    ShootButton.onTrue(
        // Old
        //  Commands.sequence(
        //      Commands.runOnce(shootsubsystem::setShootSpeedHigh, shootsubsystem),
        //      Commands.waitSeconds(1.5), // wait for spin up
        //      Commands.runOnce(intakesubsystem::SetElevatorOnShoot, intakesubsystem),
        //      Commands.waitSeconds(1.0), // wait for shot
        //      Commands.runOnce(shootsubsystem::stopshooter, shootsubsystem)));
        // New
        Commands.sequence(
            Commands.parallel(
                Commands.runOnce(shootsubsystem::setShootSpeedHigh, shootsubsystem),
                Commands.waitSeconds(1.0)), // wait for spin up
            Commands.parallel(
                Commands.runOnce(shootsubsystem::setShootSpeedHigh, shootsubsystem),
                Commands.runOnce(intakesubsystem::SetElevatorOnShoot, intakesubsystem),
                Commands.waitSeconds(1.5)), // wait for shot
            Commands.runOnce(intakesubsystem::SetElevatorOff, intakesubsystem),
            Commands.runOnce(shootsubsystem::stopshooter, shootsubsystem)));

    // Shoot Low with shelf
    ShootButtonLow.onTrue(
        Commands.sequence(
            Commands.runOnce(() -> this.shelfextend.set(true)),
            Commands.runOnce(shootsubsystem::setShootSpeedLow, shootsubsystem),
            Commands.waitSeconds(1.5), // wait for spin up
            Commands.runOnce(intakesubsystem::SetElevatorOnShoot, intakesubsystem),
            Commands.waitSeconds(1.0), // wait for shot
            Commands.runOnce(shootsubsystem::stopshooter, shootsubsystem),
            Commands.runOnce(() -> this.shelfextend.set(false))));

    // Shoot Low with shelf
    ShootButtonLow2.onTrue(
        Commands.sequence(
            Commands.runOnce(shootsubsystem::setShootSpeedLow2, shootsubsystem),
            Commands.waitSeconds(1.5), // wait for spin up
            Commands.runOnce(intakesubsystem::SetElevatorOnShoot, intakesubsystem),
            Commands.waitSeconds(1.0), // wait for shot
            Commands.runOnce(shootsubsystem::stopshooter, shootsubsystem)));

    // x-stance
    XStanceButton.onTrue(Commands.runOnce(drivetrain::enableXstance, drivetrain));
    XStanceButton.onTrue(Commands.runOnce(drivetrain::setBrakeOn, drivetrain));

    XStanceButton.onTrue(Commands.runOnce(drivetrain::disableXstance, drivetrain));
    XStanceButton.onTrue(Commands.runOnce(drivetrain::setBrakeOff, drivetrain));
    // arm extend
    // Extendarm.onTrue(Commands.runOnce(drivetrain::enableXstance, drivetrain));
    // Extendarm.onTrue(Commands.runOnce(armcontrol::setExtendArm, armcontrol));
    ResetGyroButton.onTrue(Commands.runOnce(drivetrain::zeroGyroscope, drivetrain));
  }

  // ** Use this method to define your commands for autonomous mode.

  private void configureAutoCommands() {
    AUTO_EVENT_MAP.put("event1", Commands.print("passed marker 1"));
    AUTO_EVENT_MAP.put("event2", Commands.print("passed marker 2"));

    // Waypoints
    NamedCommands.registerCommand("command1", Commands.print("passed marker 1"));
    NamedCommands.registerCommand("command2", Commands.print("passed marker 2"));
    NamedCommands.registerCommand(
        "enableXStance", Commands.runOnce(drivetrain::enableXstance, drivetrain));
    NamedCommands.registerCommand(
        "disableXStance", Commands.runOnce(drivetrain::disableXstance, drivetrain));
    NamedCommands.registerCommand("wait5Seconds", Commands.waitSeconds(5.0));
    NamedCommands.registerCommand(
        "shoothigh", Commands.runOnce(shootsubsystem::setShootSpeedHigh, shootsubsystem));
    NamedCommands.registerCommand(
        "shootstop", Commands.runOnce(shootsubsystem::stopshooter, shootsubsystem));
    NamedCommands.registerCommand(
        "elon", Commands.runOnce(intakesubsystem::SetElevatorOnShoot, intakesubsystem));
    NamedCommands.registerCommand(
        "eloff", Commands.runOnce(intakesubsystem::SetElevatorOff, intakesubsystem));
    NamedCommands.registerCommand(
        "intakeauto", Commands.runOnce(intakesubsystem::SetElevIntakeOnAuto, intakesubsystem));
    NamedCommands.registerCommand(
        "intakeoff", Commands.runOnce(intakesubsystem::SetElevatorOff, intakesubsystem));
    NamedCommands.registerCommand("rgyro", Commands.runOnce(drivetrain::zeroGyroscope, drivetrain));
    NamedCommands.registerCommand(
        "fcent", Commands.runOnce(drivetrain::enableFieldRelative, drivetrain));

    // add commands to the auto chooser
    autoChooser.addDefaultOption("Do Nothing", new InstantCommand());

    /*
     *** Close to Amp
     */
    Command BLeftRRight1 = new PathPlannerAuto("BLeftRRight1");
    autoChooser.addOption("Near AMP", BLeftRRight1);
    /*
     *** Close to Middle
     */
    Command Middle2 = new PathPlannerAuto("Middle2");
    autoChooser.addOption("Middle", Middle2);
    /*
     *** Close to Source
     */
    Command BRightRLeft3 = new PathPlannerAuto("BRightRLeft3");
    autoChooser.addOption("Near Source", BRightRLeft3);

    /*
     *** 3 note source
     */
    // Command TNoteSource = new PathPlannerAuto("TNoteSource");
    //  autoChooser.addOption("3 Note  Source", TNoteSource);

    /*
     *** 4 note source
     */
    //  Command FNoteSource = new PathPlannerAuto("FNoteSource");
    //  autoChooser.addOption("4 Note  Source", FNoteSource);

    /************
     * Test Path ************
     * demonstration of PathPlanner auto with event markers
     */
    Command autoTest = new PathPlannerAuto("TestAuto");
    autoChooser.addOption("Test Auto", autoTest);
    //
    /************
     * Choreo Test Path ************
     * demonstration of PathPlanner hosted Choreo path
     */
    Command choreoAutoTest = new PathPlannerAuto("ChoreoTest");
    autoChooser.addOption("Choreo Auto", choreoAutoTest);
    /************
     * demonstration of PathPlanner auto with event markers
     */
    Command startPoint =
        Commands.runOnce(
            () ->
                drivetrain.resetPose(
                    PathPlannerPath.fromPathFile("StartPoint").getPreviewStartingHolonomicPose()),
            drivetrain);
    autoChooser.addOption("Start Point", startPoint);

    /************
     * Drive Characterization ************
     * useful for characterizing the swerve modules for driving (i.e, determining kS
     * and kV)
     */

    /************
     * Distance Test ************
     * used for empirically determining the wheel diameter
     */
    Command distanceTestPathCommand = new PathPlannerAuto("DistanceTest");
    autoChooser.addOption("Distance Path", distanceTestPathCommand);

    /************
     * Auto Tuning ************
     * useful for tuning the autonomous PID controllers
     */
    Command tuningCommand = new PathPlannerAuto("Tuning");
    autoChooser.addOption("Auto Tuning", tuningCommand);
    Shuffleboard.getTab("MAIN").add(autoChooser.getSendableChooser());
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.get();
  }
}
