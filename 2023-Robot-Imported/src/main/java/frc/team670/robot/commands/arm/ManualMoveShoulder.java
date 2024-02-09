package frc.team670.robot.commands.arm;

import java.util.HashMap;
import java.util.Map;

import edu.wpi.first.wpilibj2.command.Command;
import frc.team670.mustanglib.commands.MustangCommand;
import frc.team670.mustanglib.subsystems.MustangSubsystemBase;
import frc.team670.mustanglib.subsystems.MustangSubsystemBase.HealthState;
import frc.team670.mustanglib.utils.MustangController;
import frc.team670.robot.subsystems.arm.Arm;

public class ManualMoveShoulder extends Command implements MustangCommand {

    private MustangController controller;
    private Arm arm;

    private HashMap<MustangSubsystemBase, HealthState> healthReqs = new HashMap<MustangSubsystemBase, HealthState>();

    public ManualMoveShoulder(Arm arm, MustangController controller) {

        this.arm = arm;
        this.controller = controller;
        healthReqs = new HashMap<MustangSubsystemBase, HealthState>();
        healthReqs.put(arm, HealthState.GREEN);
        addRequirements(arm);

    }

    @Override
    public void execute() {

        // arm.getShoulder().setOffset(arm.getShoulder().getOffset() +
        // controller.getRightY() * 5);

    }

    @Override
    public boolean isFinished() {
        return true;

    }

    @Override
    public Map<MustangSubsystemBase, HealthState> getHealthRequirements() {
        return healthReqs;
    }

}
