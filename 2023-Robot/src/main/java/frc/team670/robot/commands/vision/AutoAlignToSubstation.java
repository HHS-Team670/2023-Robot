package frc.team670.robot.commands.vision;

import java.lang.reflect.Field;
import java.util.ArrayList;
import java.util.List;
import java.util.Map;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.team670.mustanglib.commands.MustangCommand;
import frc.team670.mustanglib.commands.MustangScheduler;
import frc.team670.mustanglib.subsystems.MustangSubsystemBase;
import frc.team670.mustanglib.subsystems.MustangSubsystemBase.HealthState;
import frc.team670.robot.commands.drivebase.MoveToPose;
import frc.team670.robot.constants.FieldConstants;
import frc.team670.robot.subsystems.DriveBase;

public class AutoAlignToSubstation extends InstantCommand implements MustangCommand {

    private DriveBase driveBase;
    private MoveToPose moveCommand;
    private boolean singleOrDouble; // single substation - true, double substation - false

    public AutoAlignToSubstation(DriveBase driveBase, boolean singleOrDouble) {
        this.driveBase = driveBase;
        this.singleOrDouble = singleOrDouble;
    }

    @Override
    public Map<MustangSubsystemBase, HealthState> getHealthRequirements() {
        return null;
    }

    @Override
    public void execute() {
        if (singleOrDouble) {
            moveCommand = new MoveToPose(driveBase, (FieldConstants.allianceOrientedAllianceFlip(FieldConstants.LoadingZone.IntakePoses[0])));
        }
        else {
            moveCommand = new MoveToPose(driveBase, (FieldConstants.allianceOrientedAllianceFlip(FieldConstants.LoadingZone.IntakePoses[1])));
        }
        MustangScheduler.getInstance().schedule(moveCommand, driveBase);
    }

}