
package frc.robot.Subsystem.Shooter.IOs;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ma5951.utils.Utils.ConvUtil;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.PortMap;
import frc.robot.Subsystem.Shooter.ShooterConstants;

public class ShooterIOReal implements ShooterIO {

  protected TalonFX leftMotor;
  protected TalonFX rightMotor;
  protected TalonFXConfiguration leftMotorConfig;
  protected TalonFXConfiguration rightMotorConfig;

  protected DigitalInput gamePieceSensor;

  private VelocityVoltage leftPID;
  private VelocityVoltage rightPID;

  private StatusSignal<Current> leftCurrent;
  private StatusSignal<AngularVelocity> leftVelocity;
  private StatusSignal<Voltage> leftApliedVolts;

  private StatusSignal<Current> rightCurrent;
  private StatusSignal<AngularVelocity> rightVelocity;
  private StatusSignal<Voltage> rightApliedVolts;



  public ShooterIOReal() {
    leftMotor = new TalonFX(PortMap.Shooter.SHOOTER_LEFT_MOTOR_ID);
    rightMotor = new TalonFX(PortMap.Shooter.SHOOTER_RIGHT_MOTOR_ID);
    gamePieceSensor = new DigitalInput(PortMap.Shooter.SHOOTER_SENSORE_ID);

    leftMotorConfig = new TalonFXConfiguration();
    rightMotorConfig = new TalonFXConfiguration();

    motorLConfig();
    motorRConfig();

    leftCurrent = leftMotor.getSupplyCurrent();
    leftVelocity = leftMotor.getVelocity();
    leftApliedVolts = leftMotor.getMotorVoltage();  

    rightCurrent = rightMotor.getSupplyCurrent();   
    rightVelocity = rightMotor.getVelocity();
    rightApliedVolts = rightMotor.getMotorVoltage();

    leftPID = new VelocityVoltage(0);
    rightPID = new VelocityVoltage(0);
  }

  public void motorLConfig() {
    leftMotorConfig.Feedback.SensorToMechanismRatio = ShooterConstants.GEAR;

    leftMotorConfig.Voltage.PeakForwardVoltage = 12;
    leftMotorConfig.Voltage.PeakReverseVoltage = -12;

    leftMotorConfig.CurrentLimits.StatorCurrentLimitEnable = ShooterConstants.IS_CURRENT_LIMIT_ENABLED;
    leftMotorConfig.CurrentLimits.StatorCurrentLimit = ShooterConstants.PEAK_CURRENT_LIMIT;
    leftMotorConfig.CurrentLimits.SupplyCurrentLowerLimit  = ShooterConstants.CONTINUES_CURRENT_LIMIT;
    leftMotorConfig.CurrentLimits.SupplyCurrentLowerTime  = ShooterConstants.PEAK_CURRENT_TIME;

    leftMotorConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    leftMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    leftMotorConfig.HardwareLimitSwitch.ForwardLimitEnable = false;
    leftMotorConfig.HardwareLimitSwitch.ReverseLimitEnable = false;

    leftMotorConfig.Slot0.kP = ShooterConstants.LEFT_kP;
    leftMotorConfig.Slot0.kI = ShooterConstants.LEFT_kI;
    leftMotorConfig.Slot0.kD = ShooterConstants.LEFT_kD;

    leftMotor.getConfigurator().apply(leftMotorConfig);
  }

  public void motorRConfig() {
    rightMotorConfig.Feedback.SensorToMechanismRatio = ShooterConstants.GEAR;

    rightMotorConfig.Voltage.PeakForwardVoltage = 12;
    rightMotorConfig.Voltage.PeakReverseVoltage = -12;

    rightMotorConfig.CurrentLimits.StatorCurrentLimitEnable = ShooterConstants.IS_CURRENT_LIMIT_ENABLED;
    rightMotorConfig.CurrentLimits.StatorCurrentLimit = ShooterConstants.PEAK_CURRENT_LIMIT;
    rightMotorConfig.CurrentLimits.SupplyCurrentLowerLimit  = ShooterConstants.CONTINUES_CURRENT_LIMIT;
    rightMotorConfig.CurrentLimits.SupplyCurrentLowerTime  = ShooterConstants.PEAK_CURRENT_TIME;

    rightMotorConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    rightMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    rightMotorConfig.HardwareLimitSwitch.ForwardLimitEnable = false;
    rightMotorConfig.HardwareLimitSwitch.ReverseLimitEnable = false;

    rightMotorConfig.Slot0.kP = ShooterConstants.RIGHT_kP;
    rightMotorConfig.Slot0.kI = ShooterConstants.RIGHT_kI;
    rightMotorConfig.Slot0.kD = ShooterConstants.RIGHT_kD;

    rightMotor.getConfigurator().apply(rightMotorConfig);
    
  }

  public boolean isGamePiece() {    
    return gamePieceSensor.get();
  }

  public void setNutralModeLeft(boolean isBrake) {
  }
    
  public void setNutralModeRight(boolean isBrake) {
  }

  public void setLeftVoltage(double volt) {
    leftMotor.setVoltage(volt);
  }

  public void setRightVoltage(double volt) {
    rightMotor.setVoltage(volt);
  }

  public double getLeftCurrent() {
    return leftCurrent.getValueAsDouble();
  }

  public double getRightCurrent() {
     return rightCurrent.getValueAsDouble();
  }

  public double getLeftVelocity() {
    return ConvUtil.RPStoRPM(leftVelocity.getValueAsDouble());
  }

  public double getRightVelocity() {
    return ConvUtil.RPStoRPM(rightVelocity.getValueAsDouble());
  }

  public double getLeftAppliedVolts() {
    return leftApliedVolts.getValueAsDouble();
  }

  public double getRightAppliedVolts() {
    return rightApliedVolts.getValueAsDouble();
  }

  public void setLeftVelocity(double velocity, double feedforward) {
    leftMotor.setControl(leftPID.withVelocity(ConvUtil.RPMtoRPS(velocity)).withSlot(ShooterConstants.CONTROL_SLOT)
    .withFeedForward(feedforward));
  }

  public void setRightVelocity(double velocity, double feedforward) {
    rightMotor.setControl(rightPID.withVelocity(ConvUtil.RPMtoRPS(velocity)).withSlot(ShooterConstants.CONTROL_SLOT)
    .withFeedForward(feedforward));
  }

  public void updatePeriodic() {
    BaseStatusSignal.refreshAll(
      leftCurrent,
      leftVelocity,
      leftApliedVolts,
      rightCurrent,
      rightVelocity,
      rightApliedVolts
    );
  }

}
