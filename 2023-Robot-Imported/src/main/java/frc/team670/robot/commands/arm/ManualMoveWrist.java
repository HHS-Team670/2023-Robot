package frc.team670.robot.commands.arm;

import java.util.HashMap;
import java.util.Map;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.team670.mustanglib.commands.MustangCommand;
import frc.team670.mustanglib.subsystems.MustangSubsystemBase;
import frc.team670.mustanglib.subsystems.MustangSubsystemBase.HealthState;
import frc.team670.mustanglib.utils.MustangController;
import frc.team670.robot.subsystems.arm.Arm;

public class ManualMoveWrist extends InstantCommand implements MustangCommand {

    private Arm arm;
    private boolean positive;
    private HashMap<MustangSubsystemBase, HealthState> healthReqs = new HashMap<MustangSubsystemBase, HealthState>();

    public ManualMoveWrist(Arm arm, boolean positive) {

        this.arm = arm;
        this.positive = positive;
        healthReqs = new HashMap<MustangSubsystemBase, HealthState>();
        healthReqs.put(arm, HealthState.GREEN);
        addRequirements(arm);

    }

    @Override
    public void execute() {
        if (positive) {
            arm.getWrist().addOffset(5);
        } else {
            arm.getWrist().addOffset(-5);
        }

    }

    @Override
    public Map<MustangSubsystemBase, HealthState> getHealthRequirements() {
        return healthReqs;
    }

}
