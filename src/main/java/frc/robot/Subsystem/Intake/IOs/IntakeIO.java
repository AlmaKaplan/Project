
package frc.robot.Subsystem.Intake.IOs;

public interface IntakeIO {
    boolean isGamePieceinIntake();

    double getIntakeCurrent();

    double getIntakeVelocity();

    double getIntakeAppliedVolts();

    void setIntakeMotorNutralMode(boolean isBrake);

    void setIntakeVoltage(double volt);

    void updateIntakePeriodic();
}
