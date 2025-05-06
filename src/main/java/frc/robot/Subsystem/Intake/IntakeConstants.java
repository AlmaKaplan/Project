
package frc.robot.Subsystem.Intake;

import com.ma5951.utils.StateControl.StatesTypes.State;
import com.ma5951.utils.StateControl.StatesTypes.StatesConstants;

import frc.robot.Robot;
import frc.robot.Subsystem.Intake.IOs.IntakeIO;
import frc.robot.Subsystem.Intake.IOs.IntakeIOReal;
import frc.robot.Subsystem.Intake.IOs.IntakeIOSim;

public class IntakeConstants {
    public static final double GEAR = 1;
    public static final double PEAK_CURRENT_LIMIT = 35;
    public static final double CONTINUES_CURRENT_LIMIT = 15; 
    public static final double PEAK_CURRENT_TIME = 0.1; 
    public static final boolean IS_CURRENT_LIMIT_ENABLED = true;

    public static final int CONTROL_SLOT = 0;

    public static final double LEFT_kP = 1;
    public static final double LEFT_kI = 0;
    public static final double LEFT_kD = 0;

    public static final double RIGHT_kP = 1;
    public static final double RIGHT_kI = 0;
    public static final double RIGHT_kD = 0;

    public static final State IDLE = StatesConstants.IDLE;  
    public static final State INTAKE = new State("INTAKE");
    public static final State SORTING = new State("SORTING");
    
    public static final State[] SUBSYSTEM_STATES = new State[] {IDLE, SORTING, INTAKE};

    public static IntakeIO getIntakeIO() {
        if (Robot.isReal()) {
            return new IntakeIOReal();
        } else {
            return new IntakeIOSim();
        }
    }
}
