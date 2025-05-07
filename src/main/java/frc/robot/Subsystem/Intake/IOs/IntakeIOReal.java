
package frc.robot.Subsystem.Intake.IOs;

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
import frc.robot.Subsystem.Intake.IntakeConstants;

public class IntakeIOReal implements IntakeIO {

    protected TalonFX intakeMotor;
    protected TalonFXConfiguration MotorConfig;

    private DigitalInput intakeGamePieceSensor;

    private StatusSignal<Current> motorCurrent;
    private StatusSignal<AngularVelocity> motorVelocity;
    private StatusSignal<Voltage> motorApliedVolts;

    public IntakeIOReal() {
        intakeMotor = new TalonFX(PortMap.Intake.INTAKE_MOTOR_ID);
        intakeGamePieceSensor = new DigitalInput(PortMap.Intake.INTAKE_SENSORE_ID);

        motorCurrent = intakeMotor.getSupplyCurrent();
        motorVelocity = intakeMotor.getVelocity();
        motorApliedVolts = intakeMotor.getMotorVoltage();

        motorConfig();

    }

    public void motorConfig() {
        MotorConfig.Feedback.SensorToMechanismRatio = IntakeConstants.GEAR;

        MotorConfig.Voltage.PeakForwardVoltage = 12;
        MotorConfig.Voltage.PeakReverseVoltage = -12;

        MotorConfig.CurrentLimits.StatorCurrentLimitEnable = IntakeConstants.IS_CURRENT_LIMIT_ENABLED;
        MotorConfig.CurrentLimits.StatorCurrentLimit = IntakeConstants.PEAK_CURRENT_LIMIT;
        MotorConfig.CurrentLimits.SupplyCurrentLowerLimit  = IntakeConstants.CONTINUES_CURRENT_LIMIT;
        MotorConfig.CurrentLimits.SupplyCurrentLowerTime  = IntakeConstants.PEAK_CURRENT_TIME;

        MotorConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        MotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;

        MotorConfig.HardwareLimitSwitch.ForwardLimitEnable = false;
        MotorConfig.HardwareLimitSwitch.ReverseLimitEnable = false;

        intakeMotor.getConfigurator().apply(MotorConfig);
    }

    public void setIntakeVoltage(double volt) {
        intakeMotor.setVoltage(volt);
    }

    public boolean isGamePieceinIntake() {
        return intakeGamePieceSensor.get();
    }

    public double getIntakeCurrent() {
        return motorCurrent.getValueAsDouble();
    }

    public double getIntakeVelocity() {
        return ConvUtil.RPStoRPM(motorVelocity.getValueAsDouble());
    }

    public double getIntakeAppliedVolts() {
        return motorApliedVolts.getValueAsDouble();
    }

    public void setIntakeMotorNutralMode(boolean isBrake) {
        if (isBrake) {
            MotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        } else {
            MotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        }
        intakeMotor.getConfigurator().apply(MotorConfig);
    }

    public void updateIntakePeriodic() {
        StatusSignal.refreshAll(motorApliedVolts, motorCurrent, motorVelocity);
    }
}
