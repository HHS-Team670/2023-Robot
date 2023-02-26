package frc.team670.robot.commands.routines;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandGroupBase;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.team670.mustanglib.commands.MustangCommand;
import frc.team670.mustanglib.subsystems.MustangSubsystemBase;
import frc.team670.mustanglib.subsystems.MustangSubsystemBase.HealthState;
import frc.team670.mustanglib.utils.Logger;
import frc.team670.robot.commands.arm.MoveToTarget;
import frc.team670.robot.commands.claw.ClawEject;
import frc.team670.robot.commands.claw.ClawIdle;
import frc.team670.robot.commands.claw.ClawIntake;
import frc.team670.robot.subsystems.Claw;
import frc.team670.robot.subsystems.Claw.Status;

import frc.team670.robot.subsystems.arm.Arm;
import frc.team670.robot.subsystems.arm.ArmState;

//An implementation of an automatic claw decision program. To ensure that dynamic decisions are made in toggle, we have to make decisions on init. As such, we copied and pasted a clas from wpilib
public class ClawToggle extends CommandGroupBase implements MustangCommand {
    private Map<MustangSubsystemBase, HealthState> healthReqs;
    private Status status;
    private Claw claw;
    private Arm arm;

    public ClawToggle(Claw claw, Arm arm, Status status) {
        healthReqs = new HashMap<MustangSubsystemBase, HealthState>();
        healthReqs.put(claw, HealthState.GREEN);
        this.claw = claw;
        this.arm = arm;
    }

    private final List<Command> m_commands = new ArrayList<>();
    private int m_currentCommandIndex = -1;
    private boolean m_runWhenDisabled = true;
    private InterruptionBehavior m_interruptBehavior = InterruptionBehavior.kCancelSelf;

    @Override
    public void initialize() {
        Logger.consoleLog("Ran ToggleClaw with the status " + status.toString());
        m_commands.clear();
        if (status == Status.EJECTING) {
            addCommands(new ClawEject(claw), new MoveToTarget(arm, ArmState.STOWED));
            healthReqs.put(arm, HealthState.GREEN);
        } else if (status == Status.INTAKING) {
            if (!claw.isFull()) {
                addCommands(new ClawIntake(claw, true), new MoveToTarget(arm, ArmState.STOWED));
                healthReqs.put(arm, HealthState.GREEN);
            } else {
                addCommands(new ClawIntake(claw, false));
            }

        } else if (status == Status.IDLE) {
            addCommands(new ClawIdle(claw), new MoveToTarget(arm, ArmState.STOWED));
            healthReqs.put(arm, HealthState.GREEN);
        }
        m_currentCommandIndex = 0;
        if (!m_commands.isEmpty()) {
            m_commands.get(0).initialize();
        }
    }

    @Override
    public Map<MustangSubsystemBase, HealthState> getHealthRequirements() {
        return healthReqs;
    }

    @Override
    public final void addCommands(Command... commands) {
        if (m_currentCommandIndex != -1) {
            throw new IllegalStateException(
                    "Commands cannot be added to a composition while it's running");
        }

        CommandScheduler.getInstance().registerComposedCommands(commands);

        for (Command command : commands) {
            m_commands.add(command);
            m_requirements.addAll(command.getRequirements());
            m_runWhenDisabled &= command.runsWhenDisabled();
            if (command.getInterruptionBehavior() == InterruptionBehavior.kCancelSelf) {
                m_interruptBehavior = InterruptionBehavior.kCancelSelf;
            }
        }
    }

    @Override
    public final void execute() {
        if (m_commands.isEmpty()) {
            return;
        }

        Command currentCommand = m_commands.get(m_currentCommandIndex);

        currentCommand.execute();
        if (currentCommand.isFinished()) {
            currentCommand.end(false);
            m_currentCommandIndex++;
            if (m_currentCommandIndex < m_commands.size()) {
                m_commands.get(m_currentCommandIndex).initialize();
            }
        }
    }

    @Override
    public final void end(boolean interrupted) {
        if (interrupted
                && !m_commands.isEmpty()
                && m_currentCommandIndex > -1
                && m_currentCommandIndex < m_commands.size()) {
            m_commands.get(m_currentCommandIndex).end(true);
        }
        m_currentCommandIndex = -1;
    }

    @Override
    public final boolean isFinished() {
        return m_currentCommandIndex == m_commands.size();
    }

    @Override
    public boolean runsWhenDisabled() {
        return m_runWhenDisabled;
    }

    @Override
    public InterruptionBehavior getInterruptionBehavior() {
        return m_interruptBehavior;
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        super.initSendable(builder);

        builder.addIntegerProperty("index", () -> m_currentCommandIndex, null);
    }
}
