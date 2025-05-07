
package frc.robot.Subsystem.DriveTrain;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Subsystem.DriveTrain.IOs.DriveTrainIO;

public class DriveTrain extends SubsystemBase {
  public static final DriveTrainIO io = DriveTrainConstants.getDriveTrainIO();
  public static DriveTrain drive;

  public DriveTrain() {
    super();
  }

  public void setLeftDriveMotorNutralMode(boolean isBrake) {
    io.setLeftDriveMotorNutralMode(isBrake);
  }

  public void setRightDriveMotorNutralMode(boolean isBrake) {
    io.setRightDriveMotorNutralMode(isBrake);
  }

  public double getLeftDriveCurrent() {
    return io.getLeftDriveCurrent();
  }

  public double getRightDriveCurrent() {
    return io.getRightDriveCurrent();
  }

  public double getLeftDriveVelocity() {
    return io.getLeftDriveVelocity();
  }

  public double getRightDriveVelocity() {
    return io.getRightDriveVelocity();
  }

  public double getLeftDriveAppliedVolts() {
    return io.getLeftDriveAppliedVolts();
  }

  public double getRightDriveAppliedVolts() {
    return io.getRightDriveAppliedVolts();
  }

  public void setLeftControl(double leftSpeed, double feedforward) {
    io.setLeftControl(leftSpeed, feedforward);
  }

  public void setRightControl(double rightSpeed, double feedforward) {
    io.setrightControl(rightSpeed, feedforward);
  }

  public void setLeftDriveVoltage(double volt) {
    io.setLeftDriveVoltage(volt);
  }

  public void setRightDriveVoltage(double volt) {
    io.setRightDriveVoltage(volt);
  }

  public static DriveTrain getInstance() {
    if (drive == null) {
      drive = new DriveTrain();
    }
    return drive;
  }

  @Override
  public void periodic() {
    super.periodic();
    io.updatePeriodic();
  }
}
