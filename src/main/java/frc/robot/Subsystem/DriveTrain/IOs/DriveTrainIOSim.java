
package frc.robot.Subsystem.DriveTrain.IOs;

import com.ma5951.utils.ControlledMotors.Sim.TalonFXMotorSim;

import edu.wpi.first.math.system.plant.DCMotor;

public class DriveTrainIOSim extends DriveTrainIOReal {
    private TalonFXMotorSim leftMasterMotorSim;
    private TalonFXMotorSim rightMasterMotorSim;
    private TalonFXMotorSim leftSlaveMotorSim;
    private TalonFXMotorSim rightSlaveMotorSim;

    public DriveTrainIOSim() {
        super();
        leftMasterMotorSim = new TalonFXMotorSim(leftMasterMotor,leftMasterMotorConfig, DCMotor.getKrakenX60(1), 0, false);
        rightMasterMotorSim = new TalonFXMotorSim(rightMasterMotor ,rightMasterMotorConfig, DCMotor.getKrakenX60(1), 0, false);
        leftSlaveMotorSim = new TalonFXMotorSim(leftSlaveMotor,leftMasterMotorConfig, DCMotor.getKrakenX60(1), 0, false );
        rightSlaveMotorSim = new TalonFXMotorSim(rightSlaveMotor, rightMasterMotorConfig, DCMotor.getKrakenX60(1), 0, false);
    }

    @Override
    public void updatePeriodic() {
        super.updatePeriodic();
        leftMasterMotorSim.updateSim();
        rightMasterMotorSim.updateSim();
        leftSlaveMotorSim.updateSim();
        rightSlaveMotorSim.updateSim();
    }
}
