package commands.drive;

import edu.wpi.first.wpilibj2.command.CommandBase;
import subsystems.DriveTrain;

public class SetWheelbaseAngle extends CommandBase {
    private final DriveTrain dt;
    private final double angle;
    public SetWheelbaseAngle(double angle) {
        this.angle = Math.toRadians(angle);
        dt = DriveTrain.getInstance();
    }

    @Override
    public void execute() {
        dt.setAngleOfSwerveModules(angle);
    }
}
