
package frc.robot.Subsystem.Intake.IOs;

import com.ma5951.utils.ControlledMotors.Sim.TalonFXMotorSim;

import edu.wpi.first.math.system.plant.DCMotor;

public class IntakeIOSim extends IntakeIOReal {
    private TalonFXMotorSim intakeMotorSim;

    public IntakeIOSim() {
        super();
        intakeMotorSim = new TalonFXMotorSim(intakeMotor, MotorConfig, DCMotor.getKrakenX60(1), 0, false);
    }

    @Override
    public void updateIntakePeriodic() {
        super.updateIntakePeriodic();
        intakeMotorSim.updateSim();
    }
}
