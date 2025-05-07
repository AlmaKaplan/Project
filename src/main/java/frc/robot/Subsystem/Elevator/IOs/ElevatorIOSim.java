
package frc.robot.Subsystem.Elevator.IOs;

import com.ma5951.utils.ControlledMotors.Sim.TalonFXMotorSim;

import edu.wpi.first.math.system.plant.DCMotor;

public class ElevatorIOSim extends ElevatorIOReal {
    private TalonFXMotorSim elevatorMotorSim;

    public ElevatorIOSim() {
        super();
        elevatorMotorSim = new TalonFXMotorSim(elevatorMotor, elevatorMotorConfig, DCMotor.getFalcon500(1), 0, false);
    }

    public void updateElevatorPeriodic() {
        super.updatePeriodic();
        elevatorMotorSim.updateSim();
    }
}
