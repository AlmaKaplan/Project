
package frc.robot.Subsystem.Elevator.IOs;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.PortMap;
import frc.robot.Subsystem.Elevator.ElevatorConstants;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class ElevatorIOReal implements ElevatorIO {
    protected TalonFX elevatorMotor;
    protected TalonFXConfiguration elevatorMotorConfig;

    private StatusSignal<Current> elevatorCurrent;
    private StatusSignal<Angle> elevatorPosition;
    private StatusSignal<Voltage> elevatorApliedVolts;

    private PositionVoltage elevatorPID;

    public ElevatorIOReal() {
        elevatorMotor = new TalonFX(PortMap.Elevator.ELEVATOR_MOTOR_ID);
        elevatorMotorConfig = new TalonFXConfiguration();

        elevatorCurrent = elevatorMotor.getSupplyCurrent();
        elevatorPosition = elevatorMotor.getPosition();
        elevatorApliedVolts = elevatorMotor.getMotorVoltage();

        elevatorPID = new PositionVoltage(0);

        motorConfig();
    }

    private void motorConfig() {
        elevatorMotorConfig.Feedback.SensorToMechanismRatio = ElevatorConstants.GEAR;

        elevatorMotorConfig.Voltage.PeakForwardVoltage = 12;
        elevatorMotorConfig.Voltage.PeakReverseVoltage = -12;

        elevatorMotorConfig.CurrentLimits.StatorCurrentLimitEnable = ElevatorConstants.IS_CURRENT_LIMIT_ENABLED;
        elevatorMotorConfig.CurrentLimits.StatorCurrentLimit = ElevatorConstants.PEAK_CURRENT_LIMIT;
        elevatorMotorConfig.CurrentLimits.SupplyCurrentLowerLimit  = ElevatorConstants.CONTINUES_CURRENT_LIMIT;
        elevatorMotorConfig.CurrentLimits.SupplyCurrentLowerTime  = ElevatorConstants.PEAK_CURRENT_TIME;

        elevatorMotorConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        elevatorMotorConfig.MotorOutput.NeutralMode = com.ctre.phoenix6.signals.NeutralModeValue.Brake;

        elevatorMotorConfig.Slot0.kP = ElevatorConstants.LEFT_kP;
        elevatorMotorConfig.Slot0.kI = ElevatorConstants.LEFT_kI;
        elevatorMotorConfig.Slot0.kD = ElevatorConstants.LEFT_kD;


        elevatorMotor.getConfigurator().apply(elevatorMotorConfig);
    }

    public void setElevatorVoltage(double volts) {
        elevatorMotor.setVoltage(volts);
    }

    public double getElevatorCurrent() {
        return elevatorCurrent.getValueAsDouble();
    }

    public double getElevatorHight() {
        return (elevatorPosition.getValueAsDouble())/ElevatorConstants.gearDimeterPI;
    }

    public double getElevatorAppliedVolts() {
        return elevatorApliedVolts.getValueAsDouble();
    }

    public void setElevatorPosition(double position, double FEED_FORWARD) {
        elevatorMotor.setControl(
            elevatorPID.withPosition(position* ElevatorConstants.gearDimeterPI)
            .withSlot(ElevatorConstants.CONTROL_SLOT).withFeedForward(FEED_FORWARD));
    }

    public void setElevatorMotorNutralMode(boolean isBrake) {
        if (isBrake) {
            elevatorMotor.setNeutralMode(NeutralModeValue.Brake);
        } else {
            elevatorMotor.setNeutralMode(NeutralModeValue.Coast);
        }
        elevatorMotor.getConfigurator().apply(elevatorMotorConfig);
    }

    public void updatePeriodic() {
        StatusSignal.refreshAll( elevatorApliedVolts, elevatorCurrent, elevatorPosition);
    }
}
