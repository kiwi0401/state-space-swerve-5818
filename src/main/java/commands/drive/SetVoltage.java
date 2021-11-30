package commands.drive;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import subsystems.DriveTrain;

public class SetVoltage extends InstantCommand {
    private final DriveTrain dt;
    private final double v;
    public SetVoltage(double v) {
        this.dt = DriveTrain.getInstance();
        this.v = v;
    }

    @Override
    public void initialize() {
        dt.testSetVoltage(v);
    }
}
