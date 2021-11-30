package subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Logging;
import org.rivierarobotics.lib.MathUtil;
import org.rivierarobotics.lib.shuffleboard.RSTab;
import util.PositionStateSpaceModel;
import util.SystemIdentification;
import util.VelocityStateSpaceModel;

public class SwerveModule {
    private static final double WHEEL_RADIUS = 0.0508;
    private final double zero_ticks;
    private static final int ENCODER_RESOLUTION = 4096;

    private final CANSparkMax driveMotor;
    private final VelocityStateSpaceModel driveController;
    private double currDriveVoltage = 0;
    private double currSteerVoltage = 0;
    private double targetVelocity = 0;

    private final WPI_TalonSRX steeringMotor;
    private final static double STEER_MOTOR_TICK_TO_ANGLE = 2 * Math.PI / ENCODER_RESOLUTION;
    private final PositionStateSpaceModel steerController;
    private Rotation2d rotation2d = new Rotation2d(0);
    private Rotation2d targetAng = new Rotation2d(0);

    /**
     * Constructs a SwerveModule.
     *
     * @param driveMotorChannel    ID for the drive motor.
     * @param steeringMotorChannel ID for the turning motor.
     * @param zero_ticks           ticks when angle = 0
     */
    public SwerveModule(int driveMotorChannel, int steeringMotorChannel, double zero_ticks, boolean driveInverted, boolean steeringInverted) {
        this.driveMotor = new CANSparkMax(driveMotorChannel, CANSparkMaxLowLevel.MotorType.kBrushless);
        this.steeringMotor = new WPI_TalonSRX(steeringMotorChannel);
        this.zero_ticks = zero_ticks;

        driveMotor.getEncoder().setPositionConversionFactor(2 * Math.PI * WHEEL_RADIUS / ENCODER_RESOLUTION);
        driveMotor.getEncoder().setVelocityConversionFactor(2 * Math.PI * WHEEL_RADIUS / ENCODER_RESOLUTION * 10);

        SystemIdentification dmSID = new SystemIdentification(0.2, 2, 1);
        SystemIdentification tmSID = new SystemIdentification(0.5, 1, 3);

        steeringMotor.setInverted(true);

        this.driveController = new VelocityStateSpaceModel(
                dmSID, 0.1, 0.1,
                0.1, 0.0005, 12
        );

        this.steerController = new PositionStateSpaceModel(
                tmSID, 1, 1,
                0.01, 0.01, 0.1,
                0.03, 12
        );
    }

    private double clampAngle(double angle) {
        double low = -Math.PI;
        double high = Math.PI;
        angle = MathUtil.wrapToCircle(angle, 2 * Math.PI);
        if (angle > high) angle -= 2 * Math.PI;
        if (angle < low) angle += 2 * Math.PI;
        return angle;
    }

    public double getAbsoluteAngle() {
        return clampAngle(getAngle());
    }

    public double getAngle() {
        return (steeringMotor.getSensorCollection().getPulseWidthPosition() - zero_ticks) * STEER_MOTOR_TICK_TO_ANGLE;
    }

    public double getDriveVoltage() {
        return currDriveVoltage;
    }

    public double getSteerVoltage() {
        return currSteerVoltage;
    }

    public double getVelocity() {
        return driveMotor.getEncoder().getVelocity();
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(getVelocity(), new Rotation2d(clampAngle(getAngle())));
    }

    public void setDriveMotorVelocity(double metersPerSecond) {
        driveController.setVelocity(metersPerSecond);
    }

    public void setSteeringMotorAngle(double angleInRad) {
        steerController.setPosition(angleInRad);
    }

    public void setDriveMotorVoltage(double voltage) {
        currDriveVoltage = voltage;
        driveMotor.setVoltage(voltage);
    }

    public void setSteeringMotorVoltage(double voltage) {
        currSteerVoltage = voltage;
        steeringMotor.setVoltage(voltage);
    }

    public double getAngleDiff(double src, double target) {
        double diff = target - src;
        if (Math.abs(diff) <= Math.PI) return diff;

        if (diff > 0) diff -= 2 * Math.PI;
        else diff += 2 * Math.PI;

        return diff;
    }

    /**
     * Sets the desired state for the module.
     *
     * @param state Desired state with speed and angle.
     */
    public void setDesiredState(SwerveModuleState state) {
        //Update State-Space Controllers
        double targetSpeed = state.speedMetersPerSecond;
        double targetRotation = state.angle.getRadians();
        double currAng = getAngle();
        double clampedAng = clampAngle(getAngle());


        double posTarget = targetRotation + Math.PI;
        double negTarget = targetRotation - Math.PI;
        double diff = 0.0;

        if (Math.abs(getAngleDiff(clampedAng, posTarget)) <= Math.abs(getAngleDiff(clampedAng, negTarget))) {
            diff += getAngleDiff(clampedAng, posTarget);
            targetSpeed = targetRotation >= 0 ? targetSpeed : -targetSpeed;
        } else {
            diff += getAngleDiff(clampedAng, negTarget);
            targetSpeed = targetRotation <= 0 ? targetSpeed : -targetSpeed;
        }

        diff = Math.abs(getAngleDiff(clampedAng, targetRotation)) < Math.abs(diff) ? getAngleDiff(clampedAng, targetRotation) : diff;



        SmartDashboard.putNumber("ANGDIFF", Math.toDegrees(diff));

        double targetAng = currAng + diff;

        rotation2d = new Rotation2d(targetAng);
        this.targetAng = new Rotation2d(clampAngle(targetAng));
        targetVelocity = targetSpeed;

        setDriveMotorVelocity(targetSpeed);
        setSteeringMotorAngle(targetAng);
    }

    public Rotation2d getRotation2d() {
        return rotation2d;
    }

    public Rotation2d targetAng() {
        return targetAng;
    }

    public double getTargetVelocity() {
        return targetVelocity;
    }

    public double getSteerMotorVel() {
        return steeringMotor.getSensorCollection().getPulseWidthVelocity();
    }

    public void periodic() {
        var driveVoltage = driveController.getAppliedVoltage(getVelocity());
        setDriveMotorVoltage(driveVoltage);

        var turnMotorVoltage = steerController.getAppliedVoltage(getAngle());
        setSteeringMotorVoltage(turnMotorVoltage);
    }
}
