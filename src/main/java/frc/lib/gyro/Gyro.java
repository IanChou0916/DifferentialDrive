package frc.lib.gyro;

import edu.wpi.first.math.geometry.Rotation2d;

public abstract class Gyro {

    public abstract Rotation2d getAngle();

    public abstract void reset();

    public abstract void zeroGyro();
    
    public abstract void setAngle(Rotation2d angle);

    public abstract Rotation2d getRoll();

    public abstract Rotation2d getPitch();

    public abstract Rotation2d getYaw();
    
}
