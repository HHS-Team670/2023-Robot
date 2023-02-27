package frc.team670.robot.commands.vision;

import java.util.Map;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.team670.mustanglib.commands.MustangCommand;
import frc.team670.mustanglib.commands.MustangScheduler;
import frc.team670.mustanglib.subsystems.MustangSubsystemBase;
import frc.team670.mustanglib.subsystems.VisionSubsystemBase;
import frc.team670.mustanglib.subsystems.MustangSubsystemBase.HealthState;
import frc.team670.mustanglib.utils.MustangController;
import frc.team670.mustanglib.utils.math.sort.AStarSearch;
import frc.team670.robot.commands.drivebase.MoveToPose;
import frc.team670.robot.commands.drivebase.PathFindMoveToPose;
import frc.team670.robot.constants.FieldConstants;
import frc.team670.robot.constants.RobotConstants;
import frc.team670.robot.pathfinder.ObstacleAvoidanceAStarMap;
import frc.team670.robot.pathfinder.PoseNode;
import frc.team670.robot.subsystems.DriveBase;

/**
 * AutoAlign - autonomously moves the robot to a given target. If no target is
 * given, it moves to
 * the closest one.`
 */
public class AutoAlign extends CommandBase implements MustangCommand {

    private VisionSubsystemBase vision;
    private DriveBase driveBase;
    private MoveToPose moveComand;
    private int goal;
    private Pose2d[] targets = new Pose2d[12];

    MustangController controller;
    private final int CONTROLLER_RIGHT = 90;
    private final int CONTROLLER_LEFT = 90 + 180;

    /**
     * AutoAligns to the closest scoring position. Stops when driver lets go of
     * button. While
     * holding button, driver can switch to adjacent scoring locations
     */
    public AutoAlign(VisionSubsystemBase vision, DriveBase driveBase, MustangController controller) {
        this.vision = vision;
        this.driveBase = driveBase;
        this.controller = controller;
    }

    @Override
    public void initialize() {
        loadTargets();
        goal = getClosestTargetIndex();
        Pose2d goalPose = targets[goal];
        moveComand = new MoveToPose(driveBase, goalPose);
        MustangScheduler.getInstance().schedule(moveComand, driveBase);
    }

    @Override
    public void execute() {
        SmartDashboard.putNumber("Goal index", goal);

        if (controller.getPOV() == CONTROLLER_RIGHT) {
            scheduleNewMove(--goal);
        } else if (controller.getPOV() == CONTROLLER_LEFT) {
            scheduleNewMove(++goal);
        }
    }

    // only ends when auto align mapped button let go
    @Override
    public boolean isFinished() {
        return false;
    }

    // only ends when auto align mapped button let go
    @Override
    public void end(boolean interrupted) {
        if (interrupted) {
            moveComand.cancel();
        }
    }

    @Override
    public Map<MustangSubsystemBase, HealthState> getHealthRequirements() {
        // TODO Auto-generated method stub
        return null;
    }

    private void loadTargets() {
        for (int i = 0; i < FieldConstants.Grids.scoringPoses.length; i++) {
            targets[i] = FieldConstants.allianceFlip(FieldConstants.Grids.scoringPoses[i]);
        }
        for (int i = 0; i < FieldConstants.LoadingZone.IntakePoses.length; i++) {
            targets[i + FieldConstants.Grids.scoringPoses.length] = FieldConstants
                    .allianceFlip(FieldConstants.LoadingZone.IntakePoses[i]);
        }
    }

    private int getClosestTargetIndex() {
        Pose2d robotPose = driveBase.getPoseEstimator().getCurrentPose();
        int closest = 0;

        for (int i = 0; i < targets.length; i++) {
            closest = robotPose.getTranslation().getDistance(targets[closest].getTranslation()) > robotPose
                    .getTranslation()
                    .getDistance(targets[i].getTranslation()) ? i : closest;
        }
        return closest;
    }

    private void scheduleNewMove(int goal) {
        // if tries to go over/under, stays and doesn't do anything
        if (goal < 0)
            goal = 0;
        else if (goal > 11)
            goal = 11;
        else {
            moveComand.cancel();
            Pose2d goalPose = targets[goal];
            moveComand = new MoveToPose(driveBase, goalPose);
            MustangScheduler.getInstance().schedule(moveComand, driveBase);
        }
    }
}
