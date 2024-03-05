package frc.robot.Commands.DriveCommands;

import edu.wpi.first.wpilibj2.command.Command;

public class GarbageCommand extends Command {
    public GarbageCommand() {

    }

    @Override
    public void execute() {
        System.gc();
    }
}
