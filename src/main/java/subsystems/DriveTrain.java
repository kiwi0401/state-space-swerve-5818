package subsystems;

import commands.drive.SwerveControl;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.controller.HolonomicDriveController;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpiutil.math.MatBuilder;
import edu.wpi.first.wpiutil.math.Nat;
import util.Gyro;

import java.nio.file.Path;

/**
 * Represents a swerve drive style drivetrain.
 */
public class DriveTrain extends SubsystemBase {
    private static DriveTrain swerveDriveTrain;

    public static DriveTrain getInstance() {
        if (swerveDriveTrain == null) swerveDriveTrain = new DriveTrain();
        return swerveDriveTrain;
    }

    public static final double MAX_SPEED = 3.0; // m/s
    public static final double MAX_ANGULAR_SPEED = Math.PI / 2; // rad/s
    public static final double MAX_ANGULAR_ACCELERATION = 2 * Math.PI; // rad/s
    private final Gyro gyro;
    private final SwerveModule[] swerveModules = new SwerveModule[4];
    private final Translation2d[] swervePosition = new Translation2d[4];
    private final SwerveDriveKinematics swerveDriveKinematics;
    private final HolonomicDriveController holonomicDriveController;
    private final SwerveDrivePoseEstimator swerveDrivePoseEstimator;

    public DriveTrain() {
        //Position relative to center of robot -> (0,0) is the center
        swervePosition[0] = new Translation2d(0.381, 0.381); //FR
        swervePosition[1] = new Translation2d(-0.381, 0.381); //FL
        swervePosition[2] = new Translation2d(-0.381, -0.381); //BL
        swervePosition[3] = new Translation2d(0.381, -0.381); //BR

        swerveModules[0] = new SwerveModule(MotorIDs.FRONT_RIGHT_DRIVE, MotorIDs.FRONT_RIGHT_STEER, 0);
        swerveModules[1] = new SwerveModule(MotorIDs.FRONT_LEFT_DRIVE, MotorIDs.FRONT_LEFT_STEER, 0);
        swerveModules[2] = new SwerveModule(MotorIDs.BACK_LEFT_DRIVE, MotorIDs.BACK_LEFT_STEER, 0);
        swerveModules[3] = new SwerveModule(MotorIDs.BACK_RIGHT_DRIVE, MotorIDs.BACK_RIGHT_STEER, 0);

        this.gyro = new Gyro();

        this.swerveDriveKinematics = new SwerveDriveKinematics(
                swervePosition[0], swervePosition[1], swervePosition[2], swervePosition[3]
        );

        this.swerveDrivePoseEstimator = new SwerveDrivePoseEstimator(
                gyro.getRotation2d(),
                new Pose2d(0, 0, gyro.getRotation2d()),
                swerveDriveKinematics,
                //Standard deviations of model states. Increase these numbers to trust your model's state estimates less.
                //This matrix is in the form [x, y, theta]^T, with units in meters and radians.
                new MatBuilder<>(Nat.N3(), Nat.N1()).fill(2, 2, .01),
                // Standard deviations of the encoder and gyro measurements. Increase these numbers to trust sensor
                // readings from encoders and gyros less. This matrix is in the form [theta], with units in radians.
                new MatBuilder<>(Nat.N1(), Nat.N1()).fill(0.1),
                //Standard deviations of the vision measurements. Increase these numbers to trust global measurements
                //from vision less. This matrix is in the form [x, y, theta]^T, with units in meters and radians.
                new MatBuilder<>(Nat.N3(), Nat.N1()).fill(0.01, 0.01, 0.01) //Vision Measurement stdev
        );

        this.holonomicDriveController = new HolonomicDriveController(
                //PID FOR X DISTANCE (kp of 1 = 1m/s extra velocity / m of error)
                new PIDController(1, 0, 0),
                //PID FOR Y DISTANCE (kp of 1.2 = 1.2m/s extra velocity / m of error)
                new PIDController(1, 0, 0),
                //PID FOR ROTATION (kp of 1 = 1rad/s extra velocity / rad of error)
                new ProfiledPIDController(1, 0, 0,
                        new TrapezoidProfile.Constraints(MAX_ANGULAR_SPEED, MAX_ANGULAR_ACCELERATION))
        );
    }

    public void setAngleOfSwerveModules(double angle) {
        for (var m : swerveModules) {
            m.setSteeringMotorAngle(angle);
        }
    }

    public void setVelocityOfSwerveModules(double vel) {
        for (var m : swerveModules) {
            m.setDriveMotorVelocity(vel);
        }
    }

    public SwerveDriveKinematics getSwerveDriveKinematics() {
        return this.swerveDriveKinematics;
    }

    /**
     * Method to drive the robot using joystick info.
     *
     * @param xSpeed        Speed of the robot in the x direction (forward).
     * @param ySpeed        Speed of the robot in the y direction (sideways).
     * @param rot           Angular rate of the robot.
     * @param fieldRelative Whether the provided x and y speeds are relative to the field.
     */
    public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
        var swerveModuleStates = swerveDriveKinematics.toSwerveModuleStates(
                fieldRelative ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, gyro.getRotation2d())
                        : new ChassisSpeeds(xSpeed, ySpeed, rot)
        );
        SwerveDriveKinematics.normalizeWheelSpeeds(swerveModuleStates, MAX_SPEED);
        for (int i = 0; i < swerveModuleStates.length; i++) {
            swerveModules[i].setDesiredState(swerveModuleStates[i]);
        }
    }

    public void drivePath(String path) {
        try {
            String trajectoryJSON = "paths/" + path + ".wpilib.json";
            Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(trajectoryJSON);
            trajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
            swerveDrivePoseEstimator.resetPosition(trajectory.getInitialPose(), gyro.getRotation2d());
            startTime = Timer.getFPGATimestamp();
        } catch (Exception ignored) {
        }
    }

    private double startTime = Timer.getFPGATimestamp();
    Trajectory trajectory = new Trajectory();

    /**
     * Call this method periodically to follow a trajectory.
     * returns false when path is done.
     */
    public boolean followHolonomicController() {
        if (Timer.getFPGATimestamp() - startTime > trajectory.getTotalTimeSeconds()) return false;

        var state = trajectory.sample(Timer.getFPGATimestamp() - startTime);
        var controls = holonomicDriveController.calculate(
                swerveDrivePoseEstimator.getEstimatedPosition(),
                state,
                //It is possible to use custom angles here that do not correspond to pathweaver's rotation target
                state.poseMeters.getRotation()
        );

        drive(controls.vxMetersPerSecond, controls.vyMetersPerSecond, controls.omegaRadiansPerSecond, false);
        return true;
    }

    public void updateOdometry() {
        swerveDrivePoseEstimator.update(
                gyro.getRotation2d(),
                swerveModules[0].getState(),
                swerveModules[1].getState(),
                swerveModules[2].getState(),
                swerveModules[3].getState()
        );
    }

    @Override
    public void periodic() {
        updateOdometry();
        for (var m : swerveModules) m.periodic();
    }
}
