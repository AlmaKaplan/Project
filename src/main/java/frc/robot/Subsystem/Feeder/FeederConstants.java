
package frc.robot.Subsystem.Feeder;

import frc.robot.Robot;
import frc.robot.Subsystem.Feeder.IOs.FeederIO;
import frc.robot.Subsystem.Feeder.IOs.FeederIOReal;
import frc.robot.Subsystem.Feeder.IOs.FeederIOSim;

import com.ma5951.utils.StateControl.StatesTypes.State;
import com.ma5951.utils.StateControl.StatesTypes.StatesConstants;


public class FeederConstants {
    public static final double GEAR = 1;
    public static final double PEAK_CURRENT_LIMIT = 35;
    public static final double CONTINUES_CURRENT_LIMIT = 15; 
    public static final double PEAK_CURRENT_TIME = 0.1; 
    public static final boolean IS_CURRENT_LIMIT_ENABLED = true;

    public static final State IDLE = StatesConstants.IDLE;  
    public static final State FEED = new State("FEED");
    public static final State REVERSE = new State("REVERSE");
    
    public static final State[] SUBSYSTEM_STATES = new State[] {IDLE, REVERSE, FEED};



    public static FeederIO getFeederIO() {
        if (Robot.isReal()) {
            return new FeederIOReal();
        } else {
            return new FeederIOSim();
        }
    }
}
