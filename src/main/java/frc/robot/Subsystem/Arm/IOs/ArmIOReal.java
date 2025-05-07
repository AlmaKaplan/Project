
package frc.robot.Subsystem.Arm.IOs;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.PortMap;
import frc.robot.Subsystem.Arm.ArmConstants;

public class ArmIOReal implements ArmIO {
    protected TalonFX armMotor;
    protected TalonFXConfiguration armMotorConfig;

    private StatusSignal<Current> armCurrent;
    private StatusSignal<Voltage> armApliedVolts;
    private StatusSignal<Angle> armPosition;

    private PositionVoltage armPID;

    public ArmIOReal() {
        armMotor = new TalonFX(PortMap.Arm.ARM_MOTOR_ID);
        armMotorConfig = new TalonFXConfiguration();

        armCurrent = armMotor.getSupplyCurrent();
        armApliedVolts = armMotor.getMotorVoltage();
        armPosition = armMotor.getPosition();
        motorConfig();
    }

    private void motorConfig() {
        armMotorConfig.Feedback.SensorToMechanismRatio = ArmConstants.GEAR;

        armMotorConfig.Voltage.PeakForwardVoltage = 12;
        armMotorConfig.Voltage.PeakReverseVoltage = -12;

        armMotorConfig.CurrentLimits.StatorCurrentLimitEnable = ArmConstants.IS_CURRENT_LIMIT_ENABLED;
        armMotorConfig.CurrentLimits.StatorCurrentLimit = ArmConstants.PEAK_CURRENT_LIMIT;
        armMotorConfig.CurrentLimits.SupplyCurrentLowerLimit = ArmConstants.CONTINUES_CURRENT_LIMIT;
        armMotorConfig.CurrentLimits.SupplyCurrentLowerTime = ArmConstants.PEAK_CURRENT_TIME;

        armMotorConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        armMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        armMotor.getConfigurator().apply(armMotorConfig);
    }

    public double getArmCurrent() {
        return armCurrent.getValueAsDouble();
    }

    public double getArmAppliedVolts() {
        return armApliedVolts.getValueAsDouble();
    }

    public double getArmPosition() {
        return (armPosition.getValueAsDouble())%360;
    }

    public void setArmVoltage(double volts) {
        armMotor.setVoltage(volts);
    }

    public void setArmPosition(double position, double feedforward) {
        armMotor.setControl(armPID.withPosition(position).withSlot(ArmConstants.CONTROL_SLOT).withFeedForward(feedforward));
    }

    public void setArmMotorNutralMode(boolean isBrake) {
        if (isBrake) {
            armMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        } else {
            armMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        }
        armMotor.getConfigurator().apply(armMotorConfig);
    }

    public void updatePeriodic() {
        StatusSignal.refreshAll(armApliedVolts, armCurrent, armPosition);
    }
}