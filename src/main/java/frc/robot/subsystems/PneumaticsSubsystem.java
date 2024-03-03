/*----------------------------------------------------------------------------*/
/* Copyright (c) 2022 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

/** */
public class PneumaticsSubsystem extends SubsystemBase {

  private final PneumaticHub m_ph = new PneumaticHub(Constants.PH_CAN);

  private final Solenoid solenoidShelf =
      new Solenoid(PneumaticsModuleType.REVPH, Constants.PneumaticsShelf);

  /** */
  public PneumaticsSubsystem() {}

  // public void stopCompressor() {
  //  m_ph.disableCompressor();
  // }

  // public void startCompressor() {
  // m_ph.enableCompressorDigital();
  //  }

  /* *
   * extend shelf
   *
   * @return
   */
  public void extendshelf() {
    solenoidShelf.set(true);
  }

  /* *
   * clsoe shelf
   *
   * @return
   */
  public void closeshelf() {
    solenoidShelf.set(false);
  }

  @Override
  public void periodic() {
    //  This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run when in simulation

  }

  // Put methods for controlling this subsystem
  // here. Call these from Commands.

}
