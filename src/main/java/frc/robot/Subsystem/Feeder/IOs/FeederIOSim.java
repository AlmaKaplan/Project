// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystem.Feeder.IOs;

import com.ma5951.utils.ControlledMotors.Sim.TalonFXMotorSim;

import edu.wpi.first.math.system.plant.DCMotor;

/** Add your docs here. */
public class FeederIOSim extends FeederIOReal {
    private TalonFXMotorSim feederMotorSim;

    public FeederIOSim() {
        super();
        feederMotorSim = new TalonFXMotorSim(feederMotor, feederMotorConfig, DCMotor.getFalcon500(1), 0, false);
    }

    @Override
    public void updateFeederPeriodic() {
        super.updateFeederPeriodic();
        feederMotorSim.updateSim();
    }
}
