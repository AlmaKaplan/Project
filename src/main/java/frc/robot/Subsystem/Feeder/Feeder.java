
package frc.robot.Subsystem.Feeder;

import com.ma5951.utils.StateControl.Subsystems.StateControlledSubsystem;

import frc.robot.Subsystem.Feeder.IOs.FeederIO;

public class Feeder extends StateControlledSubsystem{
  public static FeederIO io = FeederConstants.getFeederIO();
  public static Feeder feeder;

  public Feeder() {
    super(FeederConstants.SUBSYSTEM_STATES, "feeder");
  }

  public boolean isGamePieceinFeeder() {
    return io.isGamePieceinFeeder();
  }

  public double getFeederCurrent() {
    return io.getFeederCurrent();
  }

  public double getFeederVelocity() {
    return io.getFeederVelocity();
  }

  public double getFeederAppliedVolts() {
    return io.getFeederAppliedVolts();
  }

  public void setNutralMode(boolean isBrake) {
    io.setFeederMotorNutralMode(isBrake);
  }

  public void setFeederVoltage(double volt) {
    io.setFeederVoltage(volt);
  }

  public static Feeder getInstance() {
    if (feeder == null) {
      feeder = new Feeder();
    }
    return feeder;
  }

  @Override
  public void periodic() {
    super.periodic();
    io.updateFeederPeriodic();
  }
}
