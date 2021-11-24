package commands.drive;

import edu.wpi.first.wpilibj2.command.InstantCommand;

public class SetVoltage extends InstantCommand {
    private final double voltage;
    public SetVoltage(double voltage) {
        this.voltage = voltage;
    }
}
