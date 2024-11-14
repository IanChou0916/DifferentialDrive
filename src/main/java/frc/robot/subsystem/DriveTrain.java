package frc.robot.subsystem;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.ControlType;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.MutableMeasure;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj.CAN;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.lib.gyro.Navx;
import static edu.wpi.first.units.MutableMeasure.mutable;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Volts;
import static frc.robot.Constants.CHASSIS_TRACKWIDTH_METERS;
import static frc.robot.Constants.DRIVE_CONTINUOUS_CURRENT_LIMIT;
import static frc.robot.Constants.DRIVE_FEEDFORWARD;
import static frc.robot.Constants.DRIVE_GEAR_RATIO;
import static frc.robot.Constants.LEFT_DRIVE_MOTOR_ID;
import static frc.robot.Constants.RIGHT_DRIVE_MOTOR_ID;
import static frc.robot.Constants.DRIVE_PID;

public class DriveTrain extends SubsystemBase {

    private final CANSparkMax leftDriveMotor = new CANSparkMax(LEFT_DRIVE_MOTOR_ID.getDeviceNumber(), CANSparkMax.MotorType.kBrushless);
    private final CANSparkMax rightDriveMotor = new CANSparkMax(RIGHT_DRIVE_MOTOR_ID.getDeviceNumber(), CANSparkMax.MotorType.kBrushless);
    public final SimpleMotorFeedforward DRIVE_FF = new SimpleMotorFeedforward(DRIVE_FEEDFORWARD[0], DRIVE_FEEDFORWARD[1], DRIVE_FEEDFORWARD[2]);
    
    private final MutableMeasure<Voltage> m_appliedVoltage = mutable(Volts.of(0));
    private final MutableMeasure<Distance> m_distance = mutable(Meters.of(0));
  // Mutable holder for unit-safe linear velocity values, persisted to avoid reallocation.
    private final MutableMeasure<Velocity<Distance>> m_velocity = mutable(MetersPerSecond.of(0));
    private final SysIdRoutine m_sysidRoutine;

    private final DifferentialDriveKinematics m_kinematics = new DifferentialDriveKinematics(CHASSIS_TRACKWIDTH_METERS);
    private final DifferentialDriveOdometry m_odometry;
    private final AHRS gyro = new AHRS();
    

    public DriveTrain(){
        new Navx(); // gyro
        configDriveMotor(leftDriveMotor, false);
        configDriveMotor(rightDriveMotor, true);
        //m_odometry = new DifferentialDriveOdometry(Navx.getAngle(),getDriveDistance(leftDriveMotor),getDriveDistance(rightDriveMotor));

        m_sysidRoutine = new SysIdRoutine(
          // Empty config defaults to 1 volt/second ramp rate and 7 volt step voltage.
          new SysIdRoutine.Config(),
          new SysIdRoutine.Mechanism(
              // Tell SysId how to plumb the driving voltage to the motors.
              (Measure<Voltage> volts) -> {
                leftDriveMotor.setVoltage(volts.in(Volts));
                rightDriveMotor.setVoltage(volts.in(Volts));
              },
              // Tell SysId how to record a frame of data for each motor on the mechanism being
              // characterized.
              log -> {
                // Record a frame for the left motors.  Since these share an encoder, we consider
                // the entire group to be one motor.
                log.motor("drive-left")
                    .voltage(
                        m_appliedVoltage.mut_replace(
                            leftDriveMotor.get() * RobotController.getBatteryVoltage(), Volts))
                    .linearPosition(m_distance.mut_replace(getDriveDistance(leftDriveMotor), Meters))
                    .linearVelocity(
                        m_velocity.mut_replace(getDriveVelocity(leftDriveMotor), MetersPerSecond));
                // Record a frame for the right motors.  Since these share an encoder, we consider
                // the entire group to be one motor.
                log.motor("drive-right")
                    .voltage(
                        m_appliedVoltage.mut_replace(
                            rightDriveMotor.get() * RobotController.getBatteryVoltage(), Volts))
                    .linearPosition(m_distance.mut_replace(getDriveDistance(rightDriveMotor), Meters))
                    .linearVelocity(
                        m_velocity.mut_replace(getDriveVelocity(rightDriveMotor), MetersPerSecond));
              },
              // Tell SysId to make generated commands require this subsystem, suffix test state in
              // WPILog with this subsystem's name ("drive")
              this));





    }
    
    private void configDriveMotor(CANSparkMax motor,boolean Inverted) { 

        motor.restoreFactoryDefaults();

        motor.clearFaults();

        motor.getEncoder().setPositionConversionFactor(DRIVE_GEAR_RATIO);
        motor.getEncoder().setVelocityConversionFactor(DRIVE_GEAR_RATIO);
        motor.getPIDController().setP(DRIVE_PID[0], 0);
        motor.getPIDController().setI(DRIVE_PID[1], 0);
        motor.getPIDController().setD(DRIVE_PID[2], 0);
        //motor.getPIDController().setFF(DRIVE_FF.calculate(0)); will be turned into Simple Motor Feedforward
        motor.setSmartCurrentLimit(DRIVE_CONTINUOUS_CURRENT_LIMIT);
        motor.setIdleMode(CANSparkMax.IdleMode.kBrake);
        motor.setInverted(Inverted);
        motor.burnFlash();        
    }
    public void setSpeeds(DifferentialDriveWheelSpeeds speeds) {
    final double leftFeedforward =
        DRIVE_FF.calculate(speeds.leftMetersPerSecond);
    final double rightFeedforward =
        DRIVE_FF.calculate(speeds.rightMetersPerSecond);
    leftDriveMotor.getPIDController().setFF(leftFeedforward);
    rightDriveMotor.getPIDController().setFF(rightFeedforward);

    leftDriveMotor.getPIDController().setReference(speeds.leftMetersPerSecond,ControlType.kVelocity);
    rightDriveMotor.getPIDController().setReference(speeds.rightMetersPerSecond,ControlType.kVelocity);

    }

   
    public ChassisSpeeds getSpeeds() {
        return m_kinematics.toChassisSpeeds(
            new DifferentialDriveWheelSpeeds(
                getDriveVelocity(leftDriveMotor), getDriveVelocity(rightDriveMotor)));
    }


    private double getDriveDistance(CANSparkMax motor){
        return motor.getEncoder().getPosition();
    }
    private double getDriveVelocity(CANSparkMax motor){
        return motor.getEncoder().getVelocity();
    }

    
}
