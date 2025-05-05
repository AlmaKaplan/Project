
package frc.robot.Subsystem.Shooter.IOs;

public interface ShooterIO {

    boolean isGamePiece();

    double getRightCurrent();

    double getLeftCurrent();

    double getRightVelocity();

    double getLeftVelocity();

    double getRightAppliedVolts();

    double getLeftAppliedVolts();

    void setNutralModeLeft(boolean isBrake);

    void setNutralModeRight(boolean isBrake);

    void setLeftVoltage(double volt);

    void setRightVoltage(double volt);

    void setRightVelocity(double velocity, double feedforward);

    void setLeftVelocity(double velocity, double feedforward);

    void updatePeriodic();

}
