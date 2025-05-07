
package frc.robot.Subsystem.Shooter.IOs;

import com.ma5951.utils.ControlledMotors.Sim.TalonFXMotorSim;

import edu.wpi.first.math.system.plant.DCMotor;

public class ShooterIOSim extends ShooterIOReal {
    private TalonFXMotorSim leftMotorSim;
    private TalonFXMotorSim rightMotorSim;


    public ShooterIOSim() {
        super();
        leftMotorSim = new TalonFXMotorSim(leftMotor, leftMotorConfig, DCMotor.getKrakenX60(1),0,false);
        rightMotorSim = new TalonFXMotorSim(rightMotor, rightMotorConfig, DCMotor.getKrakenX60(1),0,false);

    }

    @Override
    public void updatePeriodic() {
        super.updatePeriodic();
        leftMotorSim.updateSim();
        rightMotorSim.updateSim();
    }
    
}
