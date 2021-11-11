package util;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.geometry.Rotation2d;

public class Gyro {
    private final AHRS navX;

    public Gyro() {
        this.navX = new AHRS(SPI.Port.kMXP);
    }

    public double getAngle() {
        return navX.getAngle();
    }

    public Rotation2d getRotation2d() {
        return new Rotation2d(Math.toRadians(getAngle()));
    }

    public double getRate() {
        return Math.toRadians(navX.getRate());
    }

    public void setAngleAdjustment(double angle) {
        navX.reset();
        navX.setAngleAdjustment(angle);
    }

    public void resetGyro() {
        navX.reset();
    }
}
