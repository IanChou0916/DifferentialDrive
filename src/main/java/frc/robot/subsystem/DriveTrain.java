package frc.robot.subsystem;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.ControlType;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.MutableMeasure;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.units.Voltage;

import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;

import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.lib.gyro.Navx;

import static frc.robot.Constants.CHASSIS_TRACKWIDTH_METERS;
import static frc.robot.Constants.CHASSIS_WHEEL_CIRCUMFERENCE;
import static frc.robot.Constants.DRIVE_CONTINUOUS_CURRENT_LIMIT;

import static frc.robot.Constants.DRIVE_GEAR_RATIO;
import static frc.robot.Constants.LEFT_DRIVE_MOTOR_ID;
import static frc.robot.Constants.RIGHT_DRIVE_MOTOR_ID;
import static edu.wpi.first.units.MutableMeasure.mutable;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;
import static frc.robot.Constants.DRIVE_FF;

import java.util.function.Supplier;

import static frc.robot.Constants.DRIVE_PID;

public class DriveTrain extends SubsystemBase {

    private final CANSparkMax leftDriveMotor = new CANSparkMax(LEFT_DRIVE_MOTOR_ID.getDeviceNumber(), CANSparkMax.MotorType.kBrushless);
    private final CANSparkMax rightDriveMotor = new CANSparkMax(RIGHT_DRIVE_MOTOR_ID.getDeviceNumber(), CANSparkMax.MotorType.kBrushless);;
    private final AHRS gyro = new AHRS();
    private final Field2d m_fieldSim = new Field2d();

    //private final LinearSystem<N2, N2, N2> m_drivetrainSystem;
    
    //public final SimpleMotorFeedforward DRIVE_FF = new SimpleMotorFeedforward(DRIVE_FEEDFORWARD[0], DRIVE_FEEDFORWARD[1], DRIVE_FEEDFORWARD[2]);
    
    private final MutableMeasure<Voltage> m_appliedVoltage = mutable(Volts.of(0));
    private final MutableMeasure<Distance> m_distance = mutable(Meters.of(0));
  // Mutable holder for unit-safe linear velocity values, persisted to avoid reallocation.
    private final MutableMeasure<Velocity<Distance>> m_velocity = mutable(MetersPerSecond.of(0));
    private final SysIdRoutine m_sysidRoutine;

    //private final DifferentialDrive m_drive = new DifferentialDrive(leftDriveMotor, rightDriveMotor);

    private final DifferentialDriveKinematics m_kinematics;
    private final DifferentialDriveOdometry m_odometry;
    private final Measure<Velocity<Voltage>> rampRate = Volts.of(1).per(Seconds.of(1));
    private final Measure<Voltage> stepVoltage = Volts.of(5);



    public DriveTrain(){
        
        configDriveMotor(leftDriveMotor, false);
        configDriveMotor(rightDriveMotor, true);
        m_kinematics = new DifferentialDriveKinematics(CHASSIS_TRACKWIDTH_METERS);
        m_odometry = new DifferentialDriveOdometry(gyro.getRotation2d(),getDriveDistance(leftDriveMotor),getDriveDistance(rightDriveMotor));
        gyro.reset();

        m_sysidRoutine = new SysIdRoutine(
          // Empty config defaults to 1 volt/second ramp rate and 7 volt step voltage.
          new SysIdRoutine.Config( //Config Manually
            //rampRate,
            //stepVoltage,
            //Seconds.of(7.5)
          ),
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
                            leftDriveMotor.getAppliedOutput() * leftDriveMotor.getBusVoltage(), Volts))
                    .linearPosition(m_distance.mut_replace(getDriveDistance(leftDriveMotor), Meters))
                    .linearVelocity(
                        m_velocity.mut_replace(getDriveVelocity(leftDriveMotor), MetersPerSecond));
                // Record a frame for the right motors.  Since these share an encoder, we consider
                // the entire group to be one motor.
                log.motor("drive-right")
                    .voltage(
                        m_appliedVoltage.mut_replace(
                            rightDriveMotor.getAppliedOutput()*rightDriveMotor.getBusVoltage(), Volts))
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

        //motor.clearFaults();

        motor.getEncoder().setPositionConversionFactor(DRIVE_GEAR_RATIO*CHASSIS_WHEEL_CIRCUMFERENCE);
        motor.getEncoder().setVelocityConversionFactor(DRIVE_GEAR_RATIO*CHASSIS_WHEEL_CIRCUMFERENCE/60);
        motor.getPIDController().setP(DRIVE_PID[0], 0);
        motor.getPIDController().setI(DRIVE_PID[1], 0);
        motor.getPIDController().setD(DRIVE_PID[2], 0);
        //motor.getPIDController().setFF(DRIVE_FF.calculate(0));
        motor.setSmartCurrentLimit(DRIVE_CONTINUOUS_CURRENT_LIMIT);
        motor.setIdleMode(CANSparkMax.IdleMode.kBrake);
        motor.setInverted(Inverted);
        motor.burnFlash();
    }

    private void setSpeeds(DifferentialDriveWheelSpeeds speeds){
        leftDriveMotor.getPIDController().setReference(speeds.leftMetersPerSecond,ControlType.kVelocity,0,DRIVE_FF.calculate(speeds.leftMetersPerSecond));
        rightDriveMotor.getPIDController().setReference(speeds.rightMetersPerSecond,ControlType.kVelocity,0,DRIVE_FF.calculate(speeds.rightMetersPerSecond));
    }

    public void drive(double xSpeed, double rot) {
        var wheelSpeeds = m_kinematics.toWheelSpeeds(new ChassisSpeeds(xSpeed, 0.0, rot));
        setSpeeds(wheelSpeeds);
    }
    private double getDriveDistance(CANSparkMax motor) {
        return motor.getEncoder().getPosition();
    }
    private double getDriveVelocity(CANSparkMax motor) {
        return motor.getEncoder().getVelocity();
    }
    private ChassisSpeeds getSpeeds(){
        return m_kinematics.toChassisSpeeds(new DifferentialDriveWheelSpeeds(getDriveVelocity(leftDriveMotor), getDriveVelocity(rightDriveMotor)));
    }

    public void updateOdometry() {
        m_odometry.update(
            gyro.getRotation2d(), getDriveDistance(leftDriveMotor), getDriveDistance(rightDriveMotor));
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        SmartDashboard.putNumber("leftMotor", getDriveVelocity(leftDriveMotor));
        SmartDashboard.putNumber("rightMotor", getDriveVelocity(rightDriveMotor));
        m_fieldSim.setRobotPose(m_odometry.getPoseMeters());
    }

    
    public Command DriveCommand(Supplier<Double> xSpeed, Supplier<Double> rot) {
        return run(() -> drive(xSpeed.get(), rot.get())).withName("DifferentialDrive");
    }
    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return m_sysidRoutine.quasistatic(direction);
    }

    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return m_sysidRoutine.dynamic(direction);
    }
    public void zeroGyro(){
        gyro.zeroYaw();
    }

}
