package commands.drive;

import edu.wpi.first.wpilibj2.command.CommandBase;
import org.rivierarobotics.lib.MathUtil;
import subsystems.DriveTrain;
import util.Gyro;

public class SetAngle extends CommandBase {
    private final DriveTrain dt;
    private final Gyro gyro;
    private final double angle;

    public SetAngle(double angle) {
        this.dt = DriveTrain.getInstance();
        this.gyro = Gyro.getInstance();
        this.angle = angle;
        addRequirements(this.dt);
    }

    @Override
    public void execute() {
        if (angle >= 0) dt.drive(0, 0, DriveTrain.MAX_ANGULAR_SPEED, false);
        else dt.drive(0, 0, -DriveTrain.MAX_ANGULAR_SPEED, false);
    }

    @Override
    public boolean isFinished() {
        return MathUtil.isWithinTolerance(gyro.getAngle(),angle,1);
    }
}
