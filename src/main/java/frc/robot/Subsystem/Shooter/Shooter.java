
package frc.robot.Subsystem.Shooter;

import com.ma5951.utils.StateControl.Subsystems.StateControlledSubsystem;

import frc.robot.Subsystem.Shooter.IOs.ShooterIO;

public class Shooter extends StateControlledSubsystem {

  public static Shooter shooter;
  public static ShooterIO io = ShooterConstanse.getShooterIO();

  private Shooter() {
    super(ShooterConstanse.SUBSYSTEM_STATES, "shooter");
  }

  public boolean isGamePiece() {
    return io.isGamePiece();
  }

  public double getRightCurrent() {
    return io.getRightCurrent();
  }

  public double getLeftCurrent() {
    return io.getLeftCurrent();
  }

  public double getRightVelocity() {
    return io.getRightVelocity();
  }

  public double getLeftVelocity() {
    return io.getLeftVelocity();
  }

  public double getRightAppliedVolts() {
    return io.getRightAppliedVolts();
  }

  public double getLeftAppliedVolts() {
    return io.getLeftAppliedVolts();
  }

  public void setNutralModeLeft(boolean isBrake) {
    io.setNutralModeLeft(isBrake);
  }

  public void setNutralModeRight(boolean isBrake) {
    io.setNutralModeRight(isBrake);
  }

  public void setLeftVoltage(double volt) {
    io.setLeftVoltage(volt);
  }

  public void setRightVoltage(double volt) {
    io.setRightVoltage(volt);
  }

  public void setRightVelocity(double velocity, double feedforward) {
    io.setRightVelocity(velocity, feedforward);
  }

  public void setLeftVelocity(double velocity, double feedforward) {
    io.setLeftVelocity(velocity, feedforward);
  }

  public static Shooter getInstance() {
    if (shooter == null) {
      shooter = new Shooter();
    }
    return shooter;
  }

  @Override
  public void periodic() {
    super.periodic();

    io.updatePeriodic();
  }
}
