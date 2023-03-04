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
    
    private Rotation2d desiredHeading = null;
    private double MAX_VELOCITY, MAX_ANGULAR_VELOCITY;

    public XboxSwerveDriveAndTurnToAngle(SwerveDrive swerveDriveBase, MustangController controller,
            double maxVelocity, double maxAngularVelocity) {
        this.driveBase = swerveDriveBase;
        this.controller = controller;
        this.rotPIDController = new RotationController(new ProfiledPIDController(4, 0, 1, // not tuned yet
        new Constraints(RobotConstants.kMaxAngularSpeedRadiansPerSecond,
                RobotConstants.kMaxAngularSpeedRadiansPerSecondSquared)));
        this.rotPIDController.setTolerance(new Rotation2d(Units.degreesToRadians(5)));

        MAX_VELOCITY = maxVelocity;
        MAX_ANGULAR_VELOCITY = maxAngularVelocity;

        addRequirements(driveBase);
    }

    @Override
    public void execute() {
        updateSnapRotation();

        double xVel = MAX_VELOCITY * modifyAxis(-controller.getLeftY()); 
        double yVel = MAX_VELOCITY * modifyAxis(-controller.getLeftX());
        double thetaVel;
        if (desiredHeading == null) {
            thetaVel = MAX_VELOCITY * modifyAxis(-controller.getRightX());
        } else {
            thetaVel = rotPIDController.calculateRotationSpeed(driveBase.getGyroscopeRotation(), desiredHeading);
        }

        driveBase.drive(
                ChassisSpeeds.fromFieldRelativeSpeeds(xVel, yVel, thetaVel, driveBase.getGyroscopeRotation()));

    }

    private void updateSnapRotation() {
        if (desiredHeading == null) {
            if (controller.getYButtonPressed()) {
                desiredHeading = new Rotation2d(0);
            } else if (controller.getXButtonPressed()) {
                desiredHeading = new Rotation2d(Math.PI/2);
            } else if (controller.getAButtonPressed()) {
                desiredHeading = new Rotation2d(Math.PI);
            } else if (controller.getBButtonPressed()) {
                desiredHeading = new Rotation2d(3*Math.PI/2);
            }
        } else {
            if (rotPIDController.atReference() || modifyAxis(-controller.getRightX()) != 0) desiredHeading = null;
        }
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
