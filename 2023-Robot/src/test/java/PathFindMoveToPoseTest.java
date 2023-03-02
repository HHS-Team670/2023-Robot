import java.util.Arrays;
import java.util.List;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPoint;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory.State;
import frc.team670.robot.constants.FieldConstants;
import frc.team670.robot.constants.RobotConstants;
import frc.team670.robot.pathfinder.ObstacleAvoidanceAStarMap;
import frc.team670.robot.pathfinder.PoseNode;

public class PathFindMoveToPoseTest {

    // @BeforeEach
    void checkContingencyNodes() {
        FieldConstants.obstacleContingencyNodes.forEach(
            node -> {
                System.out.println(String.format("%s, %s", node.pose.getX(), node.pose.getY()));
            }
        );
    }

    // @Test
    void moveToFirstCone() {
        PathFindMoveToPoseSkeleton m = new PathFindMoveToPoseSkeleton(
                new Pose2d(12, 4.5, new Rotation2d()), FieldConstants.Grids.scoringPoses[0]);
        List<State> trajStates = m.traj.getStates();
        System.out.println("------MOVE TO FIRST CONE------");
        // System.out.println("------MOVE TO FIRST CONE: Start Pose------");
        // System.out.println(String.format("%f, %f", m.startPose.getX(), m.startPose.getY()));
        // System.out.println("------MOVE TO FIRST CONE: End Pose------");
        // System.out.println(String.format("%f, %f, %f degreees", m.endPose.getX(),
        // m.endPose.getY(), m.endPose.getRotation().getDegrees()));
        System.out.println("------MOVE TO FIRST CONE: Trajectory------");
        trajStates.forEach((state) -> {
            System.out.println(
                    String.format("%f, %f", state.poseMeters.getX(), state.poseMeters.getY()));
        });
    }

    class PathFindMoveToPoseSkeleton {
        private PoseNode endPoint;
        private PoseNode startPoint;
        private ObstacleAvoidanceAStarMap AStarMap;
        PathPlannerTrajectory traj;

        public PathFindMoveToPoseSkeleton(Pose2d startPose, Pose2d finalPose) {
            this.startPoint = new PoseNode(startPose);
            this.endPoint = new PoseNode(finalPose);
            this.AStarMap = new ObstacleAvoidanceAStarMap(startPoint, endPoint,
                    FieldConstants.obstacles, FieldConstants.obstacleContingencyNodes);

            List<PoseNode> fullPath = AStarMap.findPath();
            init(fullPath);
        }

        void init(List<PoseNode> fullPath) {
            if (fullPath == null)
                return;

            fullPath.forEach((node) -> {
                System.out.println(node.pose);
            });
            // Depending on if internal points are present, make a new array of the other
            // points in the path.
            PathPoint[] fullPathPoints = getPathPointsFromNodes(fullPath);

            traj = PathPlanner.generatePath(RobotConstants.kAutoPathConstraints,
                    Arrays.asList(fullPathPoints));
        }

        private PathPoint[] getPathPointsFromNodes(List<PoseNode> fullPath) {
            PathPoint[] fullPathPoints = new PathPoint[fullPath.size()];

            Rotation2d Heading = new Rotation2d(fullPath.get(1).getX() - startPoint.getX(),
                    fullPath.get(1).getY() - startPoint.getY());
            double totalDis = 0;
            for (int i = 0; i < fullPath.size() - 1; i++) {
                totalDis += Math.hypot(fullPath.get(i + 1).getX() - fullPath.get(i).getX(),
                        fullPath.get(i + 1).getY() - fullPath.get(i).getY());
            }

            for (int i = 0; i < fullPath.size(); i++) {
                if (i == 0) { // first node
                    fullPathPoints[i] =
                            new PathPoint(new Translation2d(startPoint.getX(), startPoint.getY()),
                                    Heading, startPoint.pose.getRotation() // driveBase.getPoseEstimator().getCurrentPose().getRotation(),
                            );
                } else if (i + 1 == fullPath.size()) { // last node
                    fullPathPoints[i] = new PathPoint(
                            new Translation2d(endPoint.getX(), endPoint.getY()),
                            new Rotation2d(fullPath.get(i).getX() - fullPath.get(i - 1).getX(),
                                    fullPath.get(i).getY() - fullPath.get(i - 1).getY()),
                            endPoint.getHolRot().rotateBy(new Rotation2d(Math.PI)));
                } else {
                    fullPathPoints[i] = new PathPoint(
                            new Translation2d(fullPath.get(i).getX(), fullPath.get(i).getY()),
                            new Rotation2d(fullPath.get(i + 1).getX() - fullPath.get(i).getX(),
                                    fullPath.get(i + 1).getY() - fullPath.get(i).getY()),
                            endPoint.getHolRot());
                }
            }
            return fullPathPoints;
        }
    }

}
