import frc.team670.robot.subsystems.arm.ArmState;

public class Eject extends SequentialCommandGroup implements MustangCommand {
    private Map<MustangSubsystemBase, HealthState> healthReqs;
    private Claw claw;
    private Arm arm;

    public Eject(Claw claw, Arm arm) {
        healthReqs = new HashMap<MustangSubsystemBase, HealthState>();
        healthReqs.put(arm, HealthState.GREEN);
        addRequirements(arm);
        addRequirements(claw);
        this.arm = arm;
        this.claw = claw;
        addCommands(new ClawEject(claw), new MoveToTarget(arm, claw, ArmState.STOWED));
    }

    @Override
    public Map<MustangSubsystemBase, HealthState> getHealthRequirements() {
        return healthReqs;
    }

}
