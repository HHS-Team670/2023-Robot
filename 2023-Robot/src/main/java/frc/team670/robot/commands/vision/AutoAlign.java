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
 * AutoAlign - autonomously moves the robot to a given target. If no target is given, it moves to
 * the closest one.`
 */
public class AutoAlign extends CommandBase implements MustangCommand {

    private VisionSubsystemBase vision;
    private DriveBase driveBase;
    private MoveToPose pathFindingCommand;
    private Translation2d goal;
    private Translation2d[] targets = FieldConstants.Grids.complexLowTranslations;

    /**
     * AutoAligns to the closest scoring position. Stops when driver lets go of button. While
     * holding button, driver can switch to adjacent scoring locations
     */
    public AutoAlign(VisionSubsystemBase vision, DriveBase driveBase) {
        this.vision = vision;
        this.driveBase = driveBase;
    }

    public AutoAlign(VisionSubsystemBase vision, DriveBase driveBase, Translation2d goal) {
        this.vision = vision;
        this.driveBase = driveBase;
        this.goal = goal;
    }

    @Override
    public void initialize() {
        if (goal == null) {
            goal = getClosestTargetPose(goal);
        }
        Pose2d goalPose = new Pose2d(goal, FieldConstants.getRobotFacingRotation());
        pathFindingCommand = new MoveToPose(driveBase, goalPose);
        MustangScheduler.getInstance().schedule(pathFindingCommand, driveBase);
    }

    @Override
    public void execute() {
        // TODO Auto-generated method stub
        super.execute();
    }

    @Override
    public boolean isFinished() {
        return (pathFindingCommand == null || !pathFindingCommand.isScheduled());
    }

    @Override
    public void end(boolean interrupted) {
        if (interrupted) {
            pathFindingCommand.cancel();
        }
    }

    @Override
    public Map<MustangSubsystemBase, HealthState> getHealthRequirements() {
        // TODO Auto-generated method stub
        return null;
    }

    private Translation2d getClosestTargetPose(Translation2d robotTrans) {
        int closest = 0;

        for (int i = 0; i < targets.length; i++) {
            closest = robotTrans.getDistance(targets[closest]) < robotTrans
                    .getDistance(FieldConstants.allianceFlip(targets[i])) ? i : closest;
        }
        return targets[closest];
    }
}
