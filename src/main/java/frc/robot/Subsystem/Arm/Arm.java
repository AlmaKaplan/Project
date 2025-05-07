
package frc.robot.Subsystem.Arm;

import com.ma5951.utils.StateControl.Subsystems.StateControlledSubsystem;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Subsystem.Arm.IOs.ArmIO;

public class Arm extends StateControlledSubsystem {
  public static final ArmIO io = ArmConstants.getArmIO();
  public static Arm arm;

  public Arm() {
    super(ArmConstants.SUBSYSTEM_STATES, "arm");
  }

  public double getArmCurrent() {
    return io.getArmCurrent();
  }

  public double getArmPosition() {
    return io.getArmPosition();
  }

  public double getArmAppliedVolts() {
    return io.getArmAppliedVolts();
  }

  public void setArmMotorNutralMode(boolean isBrake) {
    io.setArmMotorNutralMode(isBrake);
  }

  public void setArmVoltage(double volt) {
    io.setArmVoltage(volt);
  }

  public void setArmPosition(double armSpeed, double feedforward) {
    io.setArmPosition(armSpeed, feedforward);
  }

  public static Arm getInstance() {
    if (arm == null) {
      arm = new Arm();
    }
    return arm;
  }


  @Override
  public void periodic() {
    super.periodic();
    io.updatePeriodic();
  }
}
