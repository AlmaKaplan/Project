
package frc.robot.Subsystem.Elevator;

import com.ma5951.utils.StateControl.Subsystems.StateControlledSubsystem;

import frc.robot.Subsystem.Elevator.IOs.ElevatorIO;


public class Elevator extends StateControlledSubsystem {
  public static ElevatorIO io = ElevatorConstants.getElevatorIO();
  public static Elevator elevator;
  public Elevator() {
    super(ElevatorConstants.SUBSYSTEM_STATES, "elevator");
  }

  public double getElevatorCurrent() {
    return io.getElevatorCurrent();
  }

  public double getElevatorHight() {
    return io.getElevatorHight();
  }

  public double getElevatorAppliedVolts() {
    return io.getElevatorAppliedVolts();
  }

  public void setNutralMode(boolean isBrake) {
    io.setElevatorMotorNutralMode(isBrake);
  }

  public void setElevatorVoltage(double volt) {
    io.setElevatorVoltage(volt);
  }

  public void setElevatorPosition(double position, double FEED_FORWARD) {
    io.setElevatorPosition(position, FEED_FORWARD);
  }

  public static Elevator getInstance() {
    if (elevator == null) {
      elevator = new Elevator();
    }
    return elevator;
  }

  @Override
  public void periodic() {
    super.periodic();
    io.updatePeriodic();
  }
}
