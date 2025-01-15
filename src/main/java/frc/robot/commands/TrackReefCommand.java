package frc.robot.commands;

import java.util.function.Supplier;

import com.pathplanner.lib.commands.PathfindThenFollowPath;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.IOConstants;
import frc.robot.Constants.PID;
import frc.robot.Constants.SwerveConstants;
import lib.frc706.cyberlib.subsystems.LimelightHelpers;
import lib.frc706.cyberlib.subsystems.SwerveSubsystem;

public class TrackReefCommand extends Command {

    private final SwerveSubsystem swerveSubsystem;
    private final Supplier<Double> xSpdFunction, ySpdFunction;
    private final PIDController m_turningController = new PIDController(PID.kPAutoTurning, PID.kIAutoTurning, PID.kDAutoTurning);

    public TrackReefCommand(SwerveSubsystem swerveSubsystem,
        Supplier<Double> xSpdFunction, Supplier<Double> ySpdFunction) {
        this.swerveSubsystem = swerveSubsystem;
        this.xSpdFunction = xSpdFunction;
        this.ySpdFunction = ySpdFunction;
        addRequirements(swerveSubsystem);
    }

    @Override
    public void initialize() {
        LimelightHelpers.setPipelineIndex("limelight", 0);
    }

    @Override
    public void execute() {
        //Get real-time joystick inputs and angle from apriltag
        Pose2d targetPose = AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField).getTagPose(0).orElse(new Pose3d()).toPose2d();
        Pose2d robotPose = swerveSubsystem.swerveDrive.swerveDrivePoseEstimator.getEstimatedPosition();
        Transform2d targetPose_RobotSpace = new Transform2d(targetPose, robotPose);
        Rotation2d targetAngle = targetPose_RobotSpace.getRotation();
        
        double xSpeed = ySpdFunction.get();
        double ySpeed = xSpdFunction.get();
        double turningSpeed;
        
        //Apply deadband
        xSpeed = MathUtil.applyDeadband(xSpeed, IOConstants.kDriverControllerDeadband);
        ySpeed = MathUtil.applyDeadband(ySpeed, IOConstants.kDriverControllerDeadband);

        //Make driving smoother
        turningSpeed =m_turningController.calculate(LimelightHelpers.getTargetPose3d_RobotSpace("limelight").getX());
        xSpeed = Math.abs(xSpeed)*xSpeed;
        ySpeed = Math.abs(ySpeed)*ySpeed;

        x *= MathUtil.interpolate(0.15, 1, accelMultiplier);
		y *= MathUtil.interpolate(0.15, 1, accelMultiplier);
		rot *= MathUtil.interpolate(0.20, 1, accelMultiplier);//TODO:FINISH THIS

        //Construct desired chassis speeds
        ChassisSpeeds chassisSpeeds = swerveSubsystem.swerveDrive.swerveController.getTargetSpeeds(xSpeed, ySpeed, targetAngle.getRadians(), Units.degreesToRadians(swerveSubsystem.getHeading()), SwerveConstants.kMaxVelTele MathUtil.interpolate(xSpeed, ySpeed, turningSpeed));

        //Convert chassis speeds to individual module states
        //Output each module states to wheels
        swerveSubsystem.driveRobotOriented(chassisSpeeds);

    }

    @Override
    public void end(boolean interrupted) {
        swerveSubsystem.stopModules();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}