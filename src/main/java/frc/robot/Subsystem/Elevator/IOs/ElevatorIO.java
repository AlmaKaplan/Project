
package frc.robot.Subsystem.Elevator.IOs;

public interface ElevatorIO {
    double getElevatorCurrent();

    double getElevatorHight();

    double getElevatorAppliedVolts();

    void setElevatorMotorNutralMode(boolean isBrake);

    void setElevatorVoltage(double volt);

    void setElevatorPosition(double position, double FEED_FORWARD);// in meters

    void updatePeriodic();
}
