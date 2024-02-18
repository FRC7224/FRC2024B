package frc.robot.subsystems;

public class Zone {
  public double TopMotor;
  public double BottomMotor;

  public Zone(double topmotor, double bottommotor) {
    TopMotor = topmotor;
    BottomMotor = bottommotor;
  }

  public double getBottomMotor() {
    return BottomMotor;
  }

  public double getTopMotor() {
    return TopMotor;
  }
}
