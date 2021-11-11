package frc.robot;

import commands.auto.SimpleAuto;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

public class ButtonConfiguration {
    public void initTeleop() {
        new JoystickButton(JoystickConfiguration.driverLeft, 1).whenPressed(new SimpleAuto());
    }
}
