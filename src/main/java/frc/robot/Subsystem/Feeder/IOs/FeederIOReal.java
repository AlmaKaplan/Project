
package frc.robot.Subsystem.Feeder.IOs;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ma5951.utils.Utils.ConvUtil;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.PortMap;
import frc.robot.PortMap.Feeder;
import frc.robot.Subsystem.Feeder.FeederConstants;
import frc.robot.Subsystem.Intake.IOs.IntakeIO;

public class FeederIOReal implements IntakeIO{

    protected TalonFX feederMotor;
    protected TalonFXConfiguration feederMotorConfig;

    private DigitalInput feederSensor;

    private StatusSignal<Current> feederCurrent;
    private StatusSignal<AngularVelocity> feederVelocity;
    private StatusSignal<Voltage> feederApliedVolts;

    public FeederIOReal() {
        feederMotor = new TalonFX(PortMap.Feeder.FEEDER_MOTOR_ID);
        feederSensor = new DigitalInput(PortMap.Feeder.FEEDER_SENSORE_ID);

        feederMotorConfig = new TalonFXConfiguration();

        motorConfig();

        feederCurrent = feederMotor.getSupplyCurrent();
        feederVelocity = feederMotor.getVelocity();
        feederApliedVolts = feederMotor.getMotorVoltage();  
    }

    private void motorConfig() {
        feederMotorConfig.Feedback.SensorToMechanismRatio = FeederConstants.GEAR;

        feederMotorConfig.Voltage.PeakForwardVoltage = 12;
        feederMotorConfig.Voltage.PeakReverseVoltage = -12;

        feederMotorConfig.CurrentLimits.StatorCurrentLimitEnable = FeederConstants.IS_CURRENT_LIMIT_ENABLED;
        feederMotorConfig.CurrentLimits.StatorCurrentLimit = FeederConstants.PEAK_CURRENT_LIMIT;
        feederMotorConfig.CurrentLimits.SupplyCurrentLowerLimit  = FeederConstants.CONTINUES_CURRENT_LIMIT;
        feederMotorConfig.CurrentLimits.SupplyCurrentLowerTime  = FeederConstants.PEAK_CURRENT_TIME;

        feederMotorConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        feederMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        feederMotor.getConfigurator().apply(feederMotorConfig);
    }

    public boolean getFeederSensor() {
        return feederSensor.get();
    }

    public double getFeederCurrent() {
        return feederCurrent.getValueAsDouble();
    }

    public double getFeederVelocity() {
        return ConvUtil.RPStoRPM(feederVelocity.getValueAsDouble());
    }

    public double getFeederApliedVolts() {
        return feederApliedVolts.getValueAsDouble();
    }

    public void setFeederVoltage(double volt) {
        feederMotor.setVoltage(volt);
    }

    public void setFeederMotorNutralMode(boolean isBrake) {
        if (isBrake) {
            feederMotor.setNeutralMode(NeutralModeValue.Brake);
        } else {
            feederMotor.setNeutralMode(NeutralModeValue.Coast);
        }
        feederMotor.getConfigurator().apply(feederMotorConfig);
    }

    public void updateFeederPeriodic() {
        StatusSignal.refreshAll(feederApliedVolts, feederCurrent, feederVelocity);
    }
}
