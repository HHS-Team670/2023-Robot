package frc.team670.robot.commands.drivebase;

import java.util.HashMap;
import java.util.Map;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.team670.mustanglib.commands.MustangCommand;
import frc.team670.mustanglib.subsystems.MustangSubsystemBase;
import frc.team670.mustanglib.subsystems.MustangSubsystemBase.HealthState;
import frc.team670.mustanglib.subsystems.drivebase.SwerveDrive;
import frc.team670.mustanglib.utils.MustangController;
import frc.team670.robot.constants.RobotConstants;


public class XboxSwerveDriveAndTurnToAngle extends CommandBase implements MustangCommand {
    private final SwerveDrive driveBase;
    private RotationController rotPIDController;
    private MustangController controller;

    // private Rotation2d desiredHeading = null;
    private double MAX_VELOCITY, MAX_ANGULAR_VELOCITY;

    public XboxSwerveDriveAndTurnToAngle(SwerveDrive swerveDriveBase, MustangController controller,
            double maxVelocity, double maxAngularVelocity) {
        this.driveBase = swerveDriveBase;
        this.controller = controller;
        this.rotPIDController = new RotationController(new ProfiledPIDController(3.5, 0, 0, 
                new Constraints(RobotConstants.kMaxAngularSpeedRadiansPerSecond,
                        RobotConstants.kMaxAngularSpeedRadiansPerSecondSquared)));
        this.rotPIDController.setTolerance(new Rotation2d(Units.degreesToRadians(5)));
        

        MAX_VELOCITY = maxVelocity;
        MAX_ANGULAR_VELOCITY = maxAngularVelocity;

        addRequirements(driveBase);
    }

    @Override
    public void execute() {
        // clear desired heading if at the heading or joystick touched
        if (driveBase.getDesiredHeading() != null) {
            if (rotPIDController.atReference() || modifyAxis(-controller.getRightX()) != 0)
                driveBase.setDesiredHeading(null);
        }

        double xVel = MAX_VELOCITY * modifyAxis(-controller.getLeftY());
        double yVel = MAX_VELOCITY * modifyAxis(-controller.getLeftX());
        double thetaVel;

        Rotation2d desiredHeading = driveBase.getDesiredHeading();
        if (desiredHeading == null) {
            thetaVel = MAX_ANGULAR_VELOCITY * modifyAxis(-controller.getRightX());
        } else {
            thetaVel = rotPIDController.calculateRotationSpeed(driveBase.getGyroscopeRotation(),
                    desiredHeading);
        }

        driveBase.drive(ChassisSpeeds.fromFieldRelativeSpeeds(xVel, yVel, thetaVel,
                driveBase.getGyroscopeRotation()));

    }


    @Override
    public void end(boolean interrupted) {
        driveBase.drive(new ChassisSpeeds(0.0, 0.0, 0.0));
    }

    @Override
    public Map<MustangSubsystemBase, HealthState> getHealthRequirements() {
        Map<MustangSubsystemBase, HealthState> healthRequirements =
                new HashMap<MustangSubsystemBase, HealthState>();
        healthRequirements.put(driveBase, HealthState.YELLOW);
        return healthRequirements;
    }

    private static double deadband(double value, double deadband) {
        if (Math.abs(value) > deadband) {
            if (value > 0.0) {
                return (value - deadband) / (1.0 - deadband);
            } else {
                return (value + deadband) / (1.0 - deadband);
            }
        } else {
            return 0.0;
        }
    }

    private static double modifyAxis(double value) {
        // Deadband
        value = deadband(value, 0.05);
        return value;
    }


    private class RotationController {
        private Rotation2d m_rotationError = new Rotation2d();
        private Rotation2d m_rotationTolerance = new Rotation2d();

        private final ProfiledPIDController m_thetaController;

        public RotationController(ProfiledPIDController thetaController) {
            m_thetaController = thetaController;
            m_thetaController.enableContinuousInput(0, Units.degreesToRadians(360.0));
        }

        public boolean atReference() {
            // final var eTranslate = m_poseError.getTranslation();
            final var eRotate = m_rotationError;
            // final var tolTranslate = m_poseTolerance.getTranslation();
            return Math.abs(eRotate.getRadians()) < m_rotationTolerance.getRadians();
        }

        public void setTolerance(Rotation2d tolerance) {
            m_rotationError = tolerance;
        }


        public double calculateRotationSpeed(Rotation2d currentHeading, Rotation2d desiredHeading) {
            double thetaFF = m_thetaController.calculate(currentHeading.getRadians(),
                    desiredHeading.getRadians());

            m_rotationError = desiredHeading.minus(currentHeading);

            return thetaFF;
        }

    }

}
