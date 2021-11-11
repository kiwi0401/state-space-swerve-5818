package commands.drive;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.JoystickConfiguration;
import org.rivierarobotics.lib.MathUtil;
import subsystems.DriveTrain;

public class SwerveControl extends CommandBase {
    private final DriveTrain drivetrain;
    private final Joystick leftJoystick;
    private final Joystick rightJoystick;
    public SwerveControl() {
        this.drivetrain = DriveTrain.getInstance();
        this.leftJoystick = JoystickConfiguration.driverLeft;
        this.rightJoystick = JoystickConfiguration.driverRight;
        addRequirements(this.drivetrain);
    }

    @Override
    public void execute() {
        var xSpeed = MathUtil.wrapToCircle(leftJoystick.getY()) * DriveTrain.MAX_SPEED;
        var ySpeed = MathUtil.wrapToCircle(leftJoystick.getX()) * DriveTrain.MAX_SPEED;
        var rot = MathUtil.wrapToCircle(rightJoystick.getX()) * DriveTrain.MAX_ANGULAR_SPEED;

        drivetrain.drive(xSpeed, ySpeed, rot, true);
    }
}
