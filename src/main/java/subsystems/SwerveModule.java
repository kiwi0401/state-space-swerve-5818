package subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.SwerveModuleState;
import util.PositionStateSpaceModel;
import util.SystemIdentification;
import util.VelocityStateSpaceModel;

public class SwerveModule {
    private static final double WHEEL_RADIUS = 0.0508;
    private final double zero_ticks;
    private static final int ENCODER_RESOLUTION = 4096;

    private final CANSparkMax driveMotor;
    private final VelocityStateSpaceModel driveController;

    private final WPI_TalonSRX steeringMotor;
    private final static double STEER_MOTOR_TICK_TO_ANGLE = 2 * Math.PI / ENCODER_RESOLUTION;
    private final PositionStateSpaceModel steerController;

    /**
     * Constructs a SwerveModule.
     *
     * @param driveMotorChannel    ID for the drive motor.
     * @param steeringMotorChannel ID for the turning motor.
     * @param zero_ticks ticks when angle = 0
     */
    public SwerveModule(int driveMotorChannel, int steeringMotorChannel, double zero_ticks) {
        this.driveMotor = new CANSparkMax(driveMotorChannel, CANSparkMaxLowLevel.MotorType.kBrushless);
        this.steeringMotor = new WPI_TalonSRX(steeringMotorChannel);
        this.zero_ticks = zero_ticks;

        driveMotor.getEncoder().setPositionConversionFactor(2 * Math.PI * WHEEL_RADIUS / ENCODER_RESOLUTION * 10);
        driveMotor.getEncoder().setVelocityConversionFactor(2 * Math.PI * WHEEL_RADIUS / ENCODER_RESOLUTION * 10);

        SystemIdentification dmSID = new SystemIdentification(0.0, 0.4, 0.4);
        SystemIdentification tmSID = new SystemIdentification(0.0, 0.4, 0.4);

        this.driveController = new VelocityStateSpaceModel(
                dmSID, 0.1,0.1,
                0.1,12,12
        );

        this.steerController = new PositionStateSpaceModel(
                tmSID, 0.1,0.1,
                0.1,0.1,0.1,
                12,12
        );
    }

    private double clampAngle(double angle) {
        double low = -Math.PI;
        double high = Math.PI;
        angle %= 2 * Math.PI;
        if (angle > high) angle -= 2 * Math.PI;
        if (angle < low) angle += 2 * Math.PI;
        return angle;
    }

    private double getAngle() {
        return clampAngle((steeringMotor.getSelectedSensorPosition() - zero_ticks) * STEER_MOTOR_TICK_TO_ANGLE);
    }

    private double getVelocity() {
        return driveMotor.getEncoder().getVelocity();
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(getVelocity(), new Rotation2d(getAngle()));
    }

    private void setDriveMotorVelocity(double metersPerSecond) {
        driveController.setVelocity(metersPerSecond);
    }

    private void setSteeringMotorAngle(double angleInRad) {
        steerController.setPosition(angleInRad);
    }

    private void setDriveMotorVoltage(double voltage) {
        driveMotor.setVoltage(voltage);
    }

    private void setSteeringMotorVoltage(double voltage) {
        steeringMotor.setVoltage(voltage);
    }

    /**
     * Sets the desired state for the module.
     *
     * @param state Desired state with speed and angle.
     */
    public void setDesiredState(SwerveModuleState state) {
        //Update State-Space Controllers
        setDriveMotorVelocity(state.speedMetersPerSecond);
        setSteeringMotorAngle(state.angle.getRadians());

        var driveVoltage = driveController.getAppliedVoltage(getVelocity());
        var turnMotorVoltage = steerController.getAppliedVoltage(getAngle());
        setDriveMotorVoltage(driveVoltage);
        setSteeringMotorVoltage(turnMotorVoltage);
    }
}
