
package frc.robot.Subsystem.Intake;

import com.ma5951.utils.StateControl.Subsystems.StateControlledSubsystem;

import frc.robot.Subsystem.Intake.IOs.IntakeIO;

public class Intake extends StateControlledSubsystem {

  public static Intake intake;
  public static IntakeIO io = IntakeConstants.getIntakeIO();
  
  public Intake() {
    super(IntakeConstants.SUBSYSTEM_STATES, "shooter");
  }

  public boolean isGamePieceinIntake() {
    return io.isGamePieceinIntake();
  }

  public double getIntakeCurrent() {
    return io.getIntakeCurrent();
  }

  public double getIntakeVelocity() {
    return io.getIntakeVelocity();
  }

  public double getIntakeAppliedVolts() {
    return io.getIntakeAppliedVolts();
  }

  public void setNutralMode(boolean isBrake) {
    io.setIntakeMotorNutralMode(isBrake);
  }

  public void setIntakeVoltage(double volt) {
    io.setIntakeVoltage(volt);
  }

  public static Intake getInstance() {
    if (intake == null) {
      intake = new Intake();
    }
    return intake;
  }

  @Override
  public void periodic() {
    super.periodic();
    io.updateIntakePeriodic();
  }
}
