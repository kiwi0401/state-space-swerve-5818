package commands.auto;

import edu.wpi.first.wpilibj2.command.CommandBase;
import subsystems.DriveTrain;

public class SimpleAuto extends CommandBase {
    private final DriveTrain dt;
    public SimpleAuto() {
        this.dt = DriveTrain.getInstance();
        addRequirements(this.dt);
    }

    @Override
    public void initialize() {
        dt.drivePath("testpath");
    }

    @Override
    public boolean isFinished() {
        return !dt.followHolonomicController();
    }
}
