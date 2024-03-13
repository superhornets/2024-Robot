package frc.robot.Commands.LightCommands;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.LightSubsystem;

public class LightCommand extends Command {
    private final LightSubsystem m_lights;
    private boolean isUpdated = true;
    private final BooleanSupplier isAtSpeed;
    private final BooleanSupplier isAtTarget;
    private final BooleanSupplier hasNote;
    private boolean previousAtSpeed;
    private boolean previousAtTarget;
    private boolean previousHasNote;

    public LightCommand(LightSubsystem lights, BooleanSupplier isAtSpeed, BooleanSupplier isAtTarget,
            BooleanSupplier hasNote) {
        addRequirements(lights);
        m_lights = lights;
        this.isAtSpeed = isAtSpeed;
        this.isAtTarget = isAtTarget;
        this.hasNote = hasNote;
        previousAtSpeed = isAtSpeed.getAsBoolean();
        previousAtTarget = isAtTarget.getAsBoolean();
        previousHasNote = hasNote.getAsBoolean();
    }

    @Override
    public void execute() {
        if (previousAtSpeed != isAtSpeed.getAsBoolean() || previousAtTarget != isAtTarget.getAsBoolean()
                || previousHasNote != hasNote.getAsBoolean()) {
            previousAtSpeed = isAtSpeed.getAsBoolean();
            previousAtTarget = isAtTarget.getAsBoolean();
            previousHasNote = hasNote.getAsBoolean();
            if (isAtSpeed.getAsBoolean()) {
                m_lights.setSpeedLightsOn();
            } else {
                m_lights.setSpeedLightsOff();
            }
            if (isAtTarget.getAsBoolean()) {
                m_lights.setTargetLightsOn();

            } else if (hasNote.getAsBoolean()) {
                m_lights.setNoteLightsOn();
            } else {
                m_lights.setTargetLightsOff();
            }
            m_lights.updateLights();
        }

    }

}
