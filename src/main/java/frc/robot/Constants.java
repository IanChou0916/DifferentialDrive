package frc.robot;

import static edu.wpi.first.units.Units.Inches;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import frc.lib.util.CanDeviceId;
import frc.lib.util.Conversions;

public class Constants {

    public static final CanDeviceId LEFT_DRIVE_MOTOR_ID = new CanDeviceId(1,"rio");
    public static final CanDeviceId RIGHT_DRIVE_MOTOR_ID = new CanDeviceId(2,"rio");

    public static final int DRIVE_CONTROLLER_PORT = 0;

    public static final double DRIVE_DEADBAND = 0.05;

    public static final double[] DRIVE_PID = {0.1, 0.0, 0.0};
    public static final double[] DRIVE_FEEDFORWARD = {0.0, 0.0, 0.0};
    


    public static final double MAX_VOLTAGE = 12.0;

    public static final double MAX_VELOCITY = 4.5;
    public static final double MAX_ACCELERATION = 3.0;

    public static final double CHASSIS_TRACKWIDTH_METERS =Conversions.inchesToMeters(24.0);
    public static final double SWERVE_WHEEL_CIRCUMFERENCE = Conversions.inchesToMeters(6.0)*Math.PI;

    public static final double DRIVE_GEAR_RATIO =  8.46;

    public static final int DRIVE_CONTINUOUS_CURRENT_LIMIT = 40;
    public static final double DRIVE_PEAK_CURRENT_LIMIT = 60;
    public static final double DRIVE_PEAK_CURRENT_DURATION = 0.1;
    public static final boolean DRIVE_CURRENT_ENABLED = true;

}
