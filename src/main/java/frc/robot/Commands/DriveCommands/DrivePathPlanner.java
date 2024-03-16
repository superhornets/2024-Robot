package frc.robot.Commands.DriveCommands;

import java.nio.file.Path;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;

public class DrivePathPlanner extends Command {
    PathPlannerPath path = PathPlannerPath.fromPathFile("LineUpToAmp");
    PathConstraints constraints = new PathConstraints(
            4.8, 3.0,
            Units.degreesToRadians(540), Units.degreesToRadians(720));

    boolean isFinished;

    public DrivePathPlanner() {
        // Load the path we want to pathfind to and follow

        // Create the constraints to use while pathfinding. The constraints defined in the path will only be used for the path.
        isFinished = false;
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        isFinished = AutoBuilder.pathfindThenFollowPath(path, constraints).isFinished();
    }

    @Override
    public void end(boolean interrupted) {
    }

    @Override
    public boolean isFinished() {
        return isFinished;
    }

}
