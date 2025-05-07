
package frc.robot.Subsystem.DriveTrain;

import frc.robot.Robot;
import frc.robot.Subsystem.DriveTrain.IOs.DriveTrainIO;
import frc.robot.Subsystem.DriveTrain.IOs.DriveTrainIOReal;
import frc.robot.Subsystem.DriveTrain.IOs.DriveTrainIOSim;

public class DriveTrainConstants {
    public static final double GEAR = 1; 
    public static final double PEAK_CURRENT_LIMIT = 35;
    public static final double CONTINUES_CURRENT_LIMIT = 15; 
    public static final double PEAK_CURRENT_TIME = 0.1; 
    public static final boolean IS_CURRENT_LIMIT_ENABLED = true;

    public static final double LEFT_kP = 1;
    public static final double LEFT_kI = 0;
    public static final double LEFT_kD = 0;

    public static final double RIGHT_kP = 1;
    public static final double RIGHT_kI = 0;
    public static final double RIGHT_kD = 0;

    public static final int CONTROL_SLOT = 0;

    public static DriveTrainIO getDriveTrainIO() {
        if(Robot.isReal()) {
            return new DriveTrainIOReal();
        } else {
            return new DriveTrainIOSim();
        }
    }
}
