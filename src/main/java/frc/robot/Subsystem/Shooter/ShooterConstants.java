
package frc.robot.Subsystem.Shooter;

import com.ma5951.utils.StateControl.StatesTypes.State;

import com.ma5951.utils.StateControl.StatesTypes.StatesConstants;


import frc.robot.Robot;
import frc.robot.Subsystem.Shooter.IOs.ShooterIO;
import frc.robot.Subsystem.Shooter.IOs.ShooterIOReal;
import frc.robot.Subsystem.Shooter.IOs.ShooterIOSim;

public class ShooterConstants {
    public static final double GEAR = 1;
    public static final double PEAK_CURRENT_LIMIT = 35;
    public static final double CONTINUES_CURRENT_LIMIT = 15; 
    public static final double PEAK_CURRENT_TIME = 0.1; 
    public static final boolean IS_CURRENT_LIMIT_ENABLED = true;

    public static final State IDLE = StatesConstants.IDLE;  
    public static final State HOLD = new State("HOLD");
    public static final State SHOOT = new State("SHOOT");
    public static final State SORCE = new State("SORCE");
    public static final State SORTING = new State("SORTING");

    public static final int CONTROL_SLOT = 0;

    public static final double LEFT_kP = 1;
    public static final double LEFT_kI = 0;
    public static final double LEFT_kD = 0;

    public static final double RIGHT_kP = 1;
    public static final double RIGHT_kI = 0;
    public static final double RIGHT_kD = 0;

    public static final State[] SUBSYSTEM_STATES = new State[] {IDLE, SHOOT, SORCE, SORTING, HOLD};
    

    public static ShooterIO getShooterIO() {
        if (Robot.isReal()) {
            return new ShooterIOReal();
        } else {
            return new ShooterIOSim();
        }
    }
}
