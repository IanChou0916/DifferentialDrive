package frc.robot.subsystem;

import com.revrobotics.CANSparkMax;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.CAN;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.gyro.Navx;

import static frc.robot.Constants.DRIVE_CONTINUOUS_CURRENT_LIMIT;
import static frc.robot.Constants.DRIVE_FF;
import static frc.robot.Constants.DRIVE_GEAR_RATIO;
import static frc.robot.Constants.LEFT_DRIVE_MOTOR_ID;
import static frc.robot.Constants.RIGHT_DRIVE_MOTOR_ID;
import static frc.robot.Constants.DRIVE_PID;

public class DriveTrain extends SubsystemBase {

    private final CANSparkMax leftDriveMotor;
    private final CANSparkMax rightDriveMotor;
    

    public DriveTrain(){
        leftDriveMotor = new CANSparkMax(LEFT_DRIVE_MOTOR_ID.getDeviceNumber(), CANSparkMax.MotorType.kBrushless);
        rightDriveMotor = new CANSparkMax(RIGHT_DRIVE_MOTOR_ID.getDeviceNumber(), CANSparkMax.MotorType.kBrushless);
        new Navx(); // gyro
        configDriveMotor(leftDriveMotor, false);
        configDriveMotor(rightDriveMotor, true);
    }
    
    private void configDriveMotor(CANSparkMax motor,boolean Inverted){

        motor.restoreFactoryDefaults();

        motor.clearFaults();

        motor.getEncoder().setPositionConversionFactor(DRIVE_GEAR_RATIO);
        motor.getEncoder().setVelocityConversionFactor(DRIVE_GEAR_RATIO);
        motor.getPIDController().setP(DRIVE_PID[0], 0);
        motor.getPIDController().setI(DRIVE_PID[1], 0);
        motor.getPIDController().setD(DRIVE_PID[2], 0);
        motor.getPIDController().setFF(DRIVE_FF.calculate(0));
        motor.setSmartCurrentLimit(DRIVE_CONTINUOUS_CURRENT_LIMIT);
        motor.setIdleMode(CANSparkMax.IdleMode.kBrake);
        motor.setInverted(Inverted);
        motor.burnFlash();        
    }
}
