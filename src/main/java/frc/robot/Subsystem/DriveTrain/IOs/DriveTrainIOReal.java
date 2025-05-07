
package frc.robot.Subsystem.DriveTrain.IOs;

import java.io.ObjectInputFilter.Status;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ma5951.utils.Utils.ConvUtil;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.PortMap;
import frc.robot.Subsystem.DriveTrain.DriveTrainConstants;

public class DriveTrainIOReal implements DriveTrainIO {
    protected TalonFX leftMasterMotor;
    protected TalonFX rightMasterMotor;
    protected TalonFX leftSlaveMotor;
    protected TalonFX rightSlaveMotor;

    protected TalonFXConfiguration leftMasterMotorConfig;
    protected TalonFXConfiguration rightMasterMotorConfig;

    private StatusSignal<Current> leftMasterMotorCurrent;   
    private StatusSignal<AngularVelocity> leftMasterMotorVelocity;
    private StatusSignal<Voltage> leftMasterMotorApliedVolts;

    private StatusSignal<Current> rightMasterMotorCurrent;
    private StatusSignal<AngularVelocity> rightMasterMotorVelocity;
    private StatusSignal<Voltage> rightMasterMotorApliedVolts;

    private PositionVoltage leftMasterMotorPID;
    private PositionVoltage rightMasterMotorPID;

    public DriveTrainIOReal() {
        leftMasterMotor = new TalonFX(PortMap.DriveTrain.LEFT_MASTER_MOTOR_ID);
        rightMasterMotor = new TalonFX(PortMap.DriveTrain.RIGHT_MASTER_MOTOR_ID);
        leftSlaveMotor = new TalonFX(PortMap.DriveTrain.LEFT_SLAVE_MOTOR_ID);
        rightSlaveMotor = new TalonFX(PortMap.DriveTrain.RIGHT_SLAVE_MOTOR_ID);

        leftMasterMotorConfig = new TalonFXConfiguration();
        rightMasterMotorConfig = new TalonFXConfiguration();

        leftMasterMotorCurrent = leftMasterMotor.getSupplyCurrent();
        leftMasterMotorVelocity = leftMasterMotor.getVelocity();
        leftMasterMotorApliedVolts = leftMasterMotor.getMotorVoltage();

        rightMasterMotorCurrent = rightMasterMotor.getSupplyCurrent();
        rightMasterMotorVelocity = rightMasterMotor.getVelocity();
        rightMasterMotorApliedVolts = rightMasterMotor.getMotorVoltage();

        motorLeftConfig();
        motorRightConfig();
    } 

    private void motorLeftConfig() {
        leftMasterMotorConfig.Feedback.SensorToMechanismRatio = DriveTrainConstants.GEAR;

        leftMasterMotorConfig.Voltage.PeakForwardVoltage = 12;
        leftMasterMotorConfig.Voltage.PeakReverseVoltage = -12;

        leftMasterMotorConfig.CurrentLimits.StatorCurrentLimitEnable = DriveTrainConstants.IS_CURRENT_LIMIT_ENABLED;
        leftMasterMotorConfig.CurrentLimits.StatorCurrentLimit = DriveTrainConstants.PEAK_CURRENT_LIMIT;
        leftMasterMotorConfig.CurrentLimits.SupplyCurrentLowerLimit  = DriveTrainConstants.CONTINUES_CURRENT_LIMIT;
        leftMasterMotorConfig.CurrentLimits.SupplyCurrentLowerTime  = DriveTrainConstants.PEAK_CURRENT_TIME;

        leftMasterMotorConfig.Slot0.kP = DriveTrainConstants.LEFT_kP;
        leftMasterMotorConfig.Slot0.kI = DriveTrainConstants.LEFT_kI;
        leftMasterMotorConfig.Slot0.kD = DriveTrainConstants.LEFT_kD;

        leftMasterMotorConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        leftMasterMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        leftMasterMotor.getConfigurator().apply(leftMasterMotorConfig);
        leftSlaveMotor.getConfigurator().apply(leftMasterMotorConfig);
    }

    private void motorRightConfig() {
        rightMasterMotorConfig.Feedback.SensorToMechanismRatio = DriveTrainConstants.GEAR;

        rightMasterMotorConfig.Voltage.PeakForwardVoltage = 12;
        rightMasterMotorConfig.Voltage.PeakReverseVoltage = -12;

        rightMasterMotorConfig.CurrentLimits.StatorCurrentLimitEnable = DriveTrainConstants.IS_CURRENT_LIMIT_ENABLED;
        rightMasterMotorConfig.CurrentLimits.StatorCurrentLimit = DriveTrainConstants.PEAK_CURRENT_LIMIT;
        rightMasterMotorConfig.CurrentLimits.SupplyCurrentLowerLimit  = DriveTrainConstants.CONTINUES_CURRENT_LIMIT;
        rightMasterMotorConfig.CurrentLimits.SupplyCurrentLowerTime  = DriveTrainConstants.PEAK_CURRENT_TIME;

        rightMasterMotorConfig.Slot0.kP = DriveTrainConstants.RIGHT_kP;
        rightMasterMotorConfig.Slot0.kI = DriveTrainConstants.RIGHT_kI;
        rightMasterMotorConfig.Slot0.kD = DriveTrainConstants.RIGHT_kD;

        rightMasterMotorConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        rightMasterMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        rightMasterMotor.getConfigurator().apply(rightMasterMotorConfig);
        rightSlaveMotor.getConfigurator().apply(rightMasterMotorConfig);
    }

    public double getLeftDriveCurrent() {
        return leftMasterMotorCurrent.getValueAsDouble();
    }

    public double getRightDriveCurrent() {
        return rightMasterMotorCurrent.getValueAsDouble();
    }

    public double getLeftDriveVelocity() {
        return ConvUtil.RPStoRPM(leftMasterMotorVelocity.getValueAsDouble());
    }

    public double getRightDriveVelocity() {
        return ConvUtil.RPStoRPM(rightMasterMotorVelocity.getValueAsDouble());
    }

    public double getLeftDriveAppliedVolts() {
        return leftMasterMotorApliedVolts.getValueAsDouble();
    } 

    public double getRightDriveAppliedVolts() {
        return rightMasterMotorApliedVolts.getValueAsDouble();
    }

    public void setLeftControl(double leftSpeed, double feedforward) {
        leftMasterMotor.setControl(leftMasterMotorPID.withSlot(DriveTrainConstants.CONTROL_SLOT).withFeedForward(feedforward));
    }

    public void setrightControl(double rightSpeed, double feedforward) {
        rightMasterMotor.setControl(rightMasterMotorPID.withSlot(DriveTrainConstants.CONTROL_SLOT).withFeedForward(feedforward));
    }

    public void setLeftDriveMotorNutralMode(boolean isBrake) {
         if (isBrake) {
            leftMasterMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        } else {
            leftMasterMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        }
        leftMasterMotor.getConfigurator().apply(leftMasterMotorConfig);
        leftSlaveMotor.getConfigurator().apply(leftMasterMotorConfig);
    }

    public void setRightDriveMotorNutralMode(boolean isBrake) {
        if (isBrake) {
            rightMasterMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        } else {
            rightMasterMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        }
        rightMasterMotor.getConfigurator().apply(rightMasterMotorConfig);
        rightSlaveMotor.getConfigurator().apply(rightMasterMotorConfig);
    }

    public void setLeftDriveVoltage(double volt) {
        leftMasterMotor.setVoltage(volt);
    }

    public void setRightDriveVoltage(double volt) {
        rightMasterMotor.setVoltage(volt);
    }

    public void updatePeriodic() {
        StatusSignal.refreshAll(leftMasterMotorApliedVolts, 
        leftMasterMotorCurrent, 
        leftMasterMotorVelocity,
        rightMasterMotorApliedVolts,
        rightMasterMotorCurrent, 
        rightMasterMotorVelocity);
    }
}
