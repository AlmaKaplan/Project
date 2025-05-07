
package frc.robot.Subsystem.Feeder.IOs;


public interface FeederIO {
    boolean isGamePieceinFeeder();
    
    double getFeederCurrent();

    double getFeederVelocity();

    double getFeederAppliedVolts();

    void setFeederMotorNutralMode(boolean isBrake);

    void setFeederVoltage(double volt);

    void updateFeederPeriodic();
} 

