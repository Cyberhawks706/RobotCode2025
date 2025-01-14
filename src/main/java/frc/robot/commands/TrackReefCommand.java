package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.PID;
import frc.robot.Constants.PID.*;
import lib.frc706.cyberlib.Constants;
import lib.frc706.cyberlib.subsystems.LimelightHelpers;
import lib.frc706.cyberlib.subsystems.SwerveSubsystem;

public class TrackReefCommand extends Command {

    private final SwerveSubsystem swerveSubsystem;
    private final Supplier<Double> xSpdFunction, ySpdFunction;
    private final PIDController m_turningController = new PIDController(PID.kPAutoTurning, PID.kIAutoTurning, 0);

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
        double xSpeed = ySpdFunction.get();
        double ySpeed =xSpdFunction.get();
        double turningSpeed;
        
        //Apply deadband
        xSpeed = MathUtil.applyDeadband(xSpeed, OperatorConstants.kDriverControllerDeadband);
        ySpeed = MathUtil.applyDeadband(ySpeed, OperatorConstants.kDriverControllerDeadband);

        //Make driving smoother
        turningSpeed = -m_turningController.calculate(LimelightHelpers.getTargetPose3d_RobotSpace("limelight").getX());
        xSpeed = Math.abs(xSpeed)*xSpeed;
        ySpeed = Math.abs(ySpeed)*ySpeed;

        //Construct desired chassis speeds
        ChassisSpeeds chassisSpeeds;
        chassisSpeeds = new ChassisSpeeds(xSpeed, ySpeed, turningSpeed);

        //Convert chassis speeds to individual module states
        //Output each module states to wheels
        swerveSubsystem.driveRobotOriented(swerveSubsystem.swerveDrive.swerveController.getRawTargetSpeeds(xSpeed, ySpeed, turningSpeed));

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