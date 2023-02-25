package frc.team670.robot.commands.arm;

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
import frc.team670.robot.commands.claw.ClawIntake;
import frc.team670.robot.subsystems.Claw;
import frc.team670.robot.subsystems.arm.Arm;
import frc.team670.robot.subsystems.arm.ArmState;

/**
 * Gets a valid sequence of ArmStates between the current state and the given
 * target state,
 * and queues them one by one
 * 
 * Copies all of SequentialCommandGroup from wpilib in order to avoid having to
 * declare new InstantCommands.
 * As ugly as it is to copy an entire wpilib class, it ends up being cleaner. If
 * we just try to
 * extend SequentialCommandGroup, we don't have a way to re-run the pathfinding
 * algorithm on
 * initialize. Rather, we have to do some weird InstantCommand nonsense, which
 * then removes our ability
 * to clear the queue. For more info, ask Justin or Aditi
 * 
 * @author Aditi, Armaan, Alexander, Justin, Kedar
 */
public class MoveToTarget extends CommandGroupBase implements MustangCommand {
  private Map<MustangSubsystemBase, HealthState> healthReqs;
  private ArmState target;
  private Arm arm;
  private Claw claw;

  private final List<Command> m_commands = new ArrayList<>();
  private int m_currentCommandIndex = -1;
  private boolean m_runWhenDisabled = true;
  private InterruptionBehavior m_interruptBehavior = InterruptionBehavior.kCancelSelf;

  public MoveToTarget(Arm arm, Claw claw, ArmState target) {
    healthReqs = new HashMap<MustangSubsystemBase, HealthState>();
    healthReqs.put(arm, HealthState.GREEN);
    addRequirements(arm);
    this.arm = arm;
    this.target = target;
    this.claw = claw;
    addRequirements(claw);
  }

  @Override
  public void initialize() {
    Logger.consoleLog("Ran MoveToTarget with the target " + target.toString());
    // 1) get the valid paths from current state to the target, and add them in
    // sequence
    // 2) call get Valid Path in the arm
    // 3) then call move directly to target for each of those returned paths
    m_commands.clear();
    ArmState[] path = Arm.getValidPath(arm.getTargetState(), target);
    if (target != ArmState.STOWED && claw != null && !claw.isFull()) {
      addCommands(new ClawIntake(claw));
    }
    for (int i = 1; i < path.length; i++) {
      addCommands(new MoveDirectlyToTarget(arm, path[i]));
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