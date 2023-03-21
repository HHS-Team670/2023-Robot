package frc.team670.robot.commands.vision;

import java.util.ArrayList;
import java.util.List;
import java.util.Map;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import frc.team670.mustanglib.commands.MustangCommand;
import frc.team670.mustanglib.commands.MustangScheduler;
import frc.team670.mustanglib.subsystems.MustangSubsystemBase;
import frc.team670.mustanglib.subsystems.MustangSubsystemBase.HealthState;
import frc.team670.mustanglib.utils.MustangController;
import frc.team670.robot.commands.drivebase.MoveToPose;
import frc.team670.robot.constants.FieldConstants;
import frc.team670.robot.subsystems.DriveBase;

/**
 * AutoAlign - autonomously moves the robot to a given target. If no target is given, it moves to
 * the closest one.`
 */
public class AutoAlign extends CommandBase implements MustangCommand {
    private DriveBase driveBase;
    private MoveToPose moveCommand;
    private int goal;
    private List<Pose2d> targets = new ArrayList<>(12);
    private DIRECTION direction;

    MustangController controller;
    // private final int CONTROLLER_RIGHT = 90;
    // private final int CONTROLLER_LEFT = 90 + 180;

    public static enum DIRECTION {
        CLOSEST,
        LEFT,
        RIGHT
    }

    public AutoAlign(DriveBase driveBase, MustangController controller, DIRECTION direction) {
        this.driveBase = driveBase;
        this.controller = controller;
        this.direction = direction;
    }

    @Override
    public void initialize() {
        if (targets.isEmpty()) loadTargets();
            
        goal = getClosestTargetIndex(driveBase.getPose());
        
        boolean backUp = false;
        if (direction == DIRECTION.LEFT) {
            ++goal;
            backUp = true;
        }
        else if (direction == DIRECTION.RIGHT) {
            --goal;
            backUp = true;
        }

        Pose2d goalPose = targets.get(goal);
        SmartDashboard.putString("AUTOALIGN: END POSE", String.format("(%.2f, %.2f)", goalPose.getX(), goalPose.getY()));

        this.moveCommand = new MoveToPose(driveBase, goalPose, backUp);
        MustangScheduler.getInstance().schedule(moveCommand, driveBase);
    }

    // only ends when auto align mapped button let go
    @Override
    public boolean isFinished() {
        return moveCommand == null || !moveCommand.isScheduled();
    }

    // only ends when auto align mapped button let go
    @Override
    public void end(boolean interrupted) {
        if (interrupted) moveCommand.cancel();
    }

    @Override
    public Map<MustangSubsystemBase, HealthState> getHealthRequirements() {
        return null;
    }

    private void loadTargets() {
        for (Pose2d p : FieldConstants.Grids.scoringPoses)
            targets.add(p);
        for (Pose2d p : FieldConstants.LoadingZone.IntakePoses)
            targets.add(p);

        targets.forEach(
            p -> {
                SmartDashboard.putString(p.toString(), String.format("(%.2f, %.2f)", p.getX(), p.getY()));
            }
        );
    }

    private int getClosestTargetIndex(Pose2d robotPose) {
        int closest = 0;

        for (int i = 0; i < targets.size(); i++) {
            closest = robotPose.getTranslation()
                    .getDistance(FieldConstants.allianceFlip(targets.get(i).getTranslation())) < robotPose.getTranslation()
                            .getDistance(FieldConstants.allianceFlip(targets.get(closest).getTranslation())) ? i : closest;
        }
        return closest;
    }

    // private void scheduleNewMove(int goal) {
    //     // if tries to go over/under, stays and doesn't do anything
    //     if (goal < 0)
    //         goal = 0;
    //     else if (goal > 11)
    //         goal = 11;
    //     else {
    //         moveCommand.cancel();
    //         Pose2d goalPose = targets.get(goal);
    //         moveCommand = new MoveToPose(driveBase, goalPose, true);
    //         MustangScheduler.getInstance().schedule(moveCommand, driveBase);
    //     }
    // }
}
