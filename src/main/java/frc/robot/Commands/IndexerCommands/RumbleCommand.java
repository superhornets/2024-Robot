package frc.robot.Commands.IndexerCommands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;


public class RumbleCommand extends Command {

    private final XboxController m_rumble;

    public RumbleCommand(XboxController rumble) {
        //addRequirements(rumble);
        m_rumble = rumble;
    }

    @Override
    public void initialize() {
        m_rumble.setRumble(RumbleType.kBothRumble, 1.0);
    }

    @Override
    public void end(boolean interrupted) {
        m_rumble.setRumble(RumbleType.kBothRumble, 0);
    }

    @Override
    public boolean isFinished() {

        return false;
    }

}
