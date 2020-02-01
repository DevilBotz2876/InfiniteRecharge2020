package frc.robot.subsystem;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.util.HashMap;
import java.util.Map;
import java.util.function.DoubleSupplier;

public class RobotSubsystem extends SubsystemBase {
    private ShuffleboardLayout commandLayout, valueLayout;

    public RobotSubsystem(String name) {
        ShuffleboardTab tab = Shuffleboard.getTab(name);
        commandLayout = tab.getLayout("Commands", BuiltInLayouts.kList).withSize(7, 10)
                .withProperties(Map.of("Label position", "HIDDEN")); // hide labels for commands
        valueLayout = tab.getLayout("Values", BuiltInLayouts.kGrid);
    }

    public ShuffleboardLayout getCommandLayout() {
        return commandLayout;
    }

    public ShuffleboardLayout getValueLayout() {
        return valueLayout;
    }

    public void addCommand(CommandBase command) {
        commandLayout.add(command);
    }

    public void addValue(String name, DoubleSupplier value) {
        valueLayout.addNumber(name, value);
    }


}
