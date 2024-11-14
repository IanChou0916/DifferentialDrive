package frc.lib.gyro;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.geometry.Rotation2d;

public class Navx extends Gyro {
    private final AHRS gyro;
    public Navx(){
        gyro = new AHRS();
    }

    @Override
    public Rotation2d getAngle() {
        return Rotation2d.fromDegrees(gyro.getAngle());
    }

    @Override
    public void reset() {
        gyro.reset();
    }

    @Override
    public void zeroGyro() {
        gyro.zeroYaw();
    }

    @Override
    public void setAngle(Rotation2d angle) {
        gyro.setAngleAdjustment(angle.getDegrees());
    }
    
    @Override
    public Rotation2d getRoll() {
        return Rotation2d.fromDegrees(gyro.getRoll());
    }

    @Override
    public Rotation2d getPitch() {
        return Rotation2d.fromDegrees(gyro.getPitch());
    }

    @Override
    public Rotation2d getYaw() {
        return Rotation2d.fromDegrees(gyro.getYaw());
    }
    
    
}
