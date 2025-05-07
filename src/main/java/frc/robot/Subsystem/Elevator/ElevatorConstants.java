
package frc.robot.Subsystem.Elevator;

import com.ma5951.utils.StateControl.StatesTypes.State;
import com.ma5951.utils.StateControl.StatesTypes.StatesConstants;

import frc.robot.Robot;
import frc.robot.Subsystem.Elevator.IOs.ElevatorIO;
import frc.robot.Subsystem.Elevator.IOs.ElevatorIOReal;
import frc.robot.Subsystem.Elevator.IOs.ElevatorIOSim;


public class ElevatorConstants {
    public static final double GEAR = 1; 
    public static final double PEAK_CURRENT_LIMIT = 35;
    public static final double CONTINUES_CURRENT_LIMIT = 15; 
    public static final double PEAK_CURRENT_TIME = 0.1; 
    public static final boolean IS_CURRENT_LIMIT_ENABLED = true;

    public static final double gearDimeterPI = 0.05*2*Math.PI; // היקף במטר

    public static final double LEFT_kP = 1;
    public static final double LEFT_kI = 0;
    public static final double LEFT_kD = 0;

    public static final int CONTROL_SLOT = 0;


    public static final State IDLE = StatesConstants.IDLE;  
    public static final State L1 = new State("L1");
    public static final State L2 = new State("L2");
    public static final State L3 = new State("L3");
    public static final State L4 = new State("L4");
    
    public static final State[] SUBSYSTEM_STATES = new State[] {IDLE, L1, L2,L3,L4};

    public static ElevatorIO getElevatorIO() {
        if (Robot.isReal()) {
            return new ElevatorIOReal();
        } else {
            return new ElevatorIOSim();
        }
    }

}
