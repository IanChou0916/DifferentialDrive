package frc.lib.gyro;
import edu.wpi.first.math.geometry.Rotation2d;

public class Piegon extends Gyro {
    private final Piegon gyro;
    public Piegon(){
        gyro = new Piegon();
    }

    @Override
    public Rotation2d getAngle() {
        return gyro.getAngle();
    }

    @Override
    public void reset() {
        gyro.reset();
    }

    @Override
    public void zeroGyro() {
        gyro.zeroGyro();
    }

    @Override
    public void setAngle(Rotation2d angle) {
        gyro.setAngle(angle);
    }
    
    @Override
    public Rotation2d getRoll() {
        return gyro.getRoll();
    }

    @Override
    public Rotation2d getPitch() {
        return gyro.getPitch();
    }

    @Override
    public Rotation2d getYaw() {
        return gyro.getYaw();
    }
    
}
