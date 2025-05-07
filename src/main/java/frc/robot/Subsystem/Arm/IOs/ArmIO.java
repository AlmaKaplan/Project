
package frc.robot.Subsystem.Arm.IOs;

public interface ArmIO {
    double getArmCurrent();

    double getArmAppliedVolts();

    double getArmPosition();

    void setArmPosition(double armSpeed, double feedforward);

    void setArmMotorNutralMode(boolean isBrake);

    void setArmVoltage(double volt);

    void updatePeriodic();
}
