
package frc.robot.Subsystem.DriveTrain.IOs;

/** Add your docs here. */
public interface DriveTrainIO {
    double getLeftDriveCurrent();

    double getRightDriveCurrent();

    double getLeftDriveVelocity();

    double getRightDriveVelocity();

    double getLeftDriveAppliedVolts();

    double getRightDriveAppliedVolts();

    void setLeftControl(double leftSpeed, double feedforward);

    void setrightControl(double rightSpeed, double feedforward);

    void setLeftDriveMotorNutralMode(boolean isBrake);

    void setRightDriveMotorNutralMode(boolean isBrake);

    void setLeftDriveVoltage(double volt);

    void setRightDriveVoltage(double volt);

    void updatePeriodic();
}
