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
    private final BooleanSupplier isAtAngle;
    private final BooleanSupplier isHomed;
    private boolean previousAtSpeed;
    private boolean previousAtTarget;
    private boolean previousHasNote;
    private boolean previousAtAngle;
    private boolean previousIsHomed;

    public LightCommand(LightSubsystem lights, BooleanSupplier isAtSpeed, BooleanSupplier isAtTarget,
            BooleanSupplier hasNote, BooleanSupplier isAtAngle, BooleanSupplier isHomed) {
        addRequirements(lights);
        m_lights = lights;
        this.isAtSpeed = isAtSpeed;
        this.isAtTarget = isAtTarget;
        this.hasNote = hasNote;
        this.isAtAngle = isAtAngle;
        this.isHomed = isHomed;
        previousAtSpeed = isAtSpeed.getAsBoolean();
        previousAtTarget = isAtTarget.getAsBoolean();
        previousHasNote = hasNote.getAsBoolean();
        previousAtAngle = isAtAngle.getAsBoolean();
        previousIsHomed = isHomed.getAsBoolean();
    }

    @Override
    public void execute() {
        if (previousAtSpeed != isAtSpeed.getAsBoolean() || previousAtTarget != isAtTarget.getAsBoolean()
                || previousHasNote != hasNote.getAsBoolean() || previousAtAngle != isAtAngle.getAsBoolean()
                || previousIsHomed != isHomed.getAsBoolean()) {
            previousAtSpeed = isAtSpeed.getAsBoolean();
            previousAtTarget = isAtTarget.getAsBoolean();
            previousHasNote = hasNote.getAsBoolean();
            previousAtAngle = isAtAngle.getAsBoolean();
            previousIsHomed = isHomed.getAsBoolean();
            if (isHomed.getAsBoolean()) {
                m_lights.setDownLightsOn();
            } else if (isAtSpeed.getAsBoolean() && isAtAngle.getAsBoolean()) {
                m_lights.setSpeedAndAngleLightsOn();
            } else if (isAtSpeed.getAsBoolean()) {
                m_lights.setSpeedLightsOn();
            } else if (isAtAngle.getAsBoolean()) {
                m_lights.setAngleLightsOn();
            } else {
                m_lights.setSpeedLightsOff();
            }

            if (isAtTarget.getAsBoolean() && hasNote.getAsBoolean()) {
                m_lights.setNoteAndTargetingLightsOn();

            } else if (hasNote.getAsBoolean()) {
                m_lights.setNoteLightsOn();
            } else if (isAtTarget.getAsBoolean()) {
                m_lights.setTargetLightsOn();
            } else {
                m_lights.setTargetLightsOff();
            }
            m_lights.updateLights();
        }

    }

}
