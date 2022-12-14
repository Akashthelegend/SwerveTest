// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import com.ctre.phoenix.sensors.PigeonIMU;

import frc.robot.SwerveModule;
import frc.robot.Constants;
import frc.robot.RobotMap;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SwerveSubsystem extends SubsystemBase {
  /** Creates a new SwerveSubsystem. */
  public SwerveDriveOdometry swerveOdometry;
  public SwerveModule[] mSwerveMods;
  public PigeonIMU gyro;

  public SwerveSubsystem() {
    gyro = new PigeonIMU(RobotMap.pigeonID); //TODO set constant for pigeon
    gyro.configFactoryDefault();
    zeroGyro();

    swerveOdometry = new SwerveDriveOdometry(Constants.Swerve.swerveKinematics, getYaw()); //TODO set constant for kinematics

        mSwerveMods = new SwerveModule[] {
            new SwerveModule(0, RobotMap.Mod0.constants), //TODO set constants for each module
            new SwerveModule(1, RobotMap.Mod1.constants),
            new SwerveModule(2, RobotMap.Mod2.constants),
            new SwerveModule(3, RobotMap.Mod3.constants)
        };
  }

  public void drive(Translation2d translation, double rotation, boolean fieldRelative, boolean isOpenLoop) {
    SwerveModuleState[] swerveModuleStates =
        Constants.Swerve.swerveKinematics.toSwerveModuleStates(
            fieldRelative ? ChassisSpeeds.fromFieldRelativeSpeeds(
                                translation.getX(), 
                                translation.getY(), 
                                rotation, 
                                getYaw()
                            )
                            : new ChassisSpeeds(
                                translation.getX(), 
                                translation.getY(), 
                                rotation)
                            );
    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Constants.Swerve.maxSpeed);

    for(SwerveModule mod : mSwerveMods){
        mod.setDesiredState(swerveModuleStates[mod.moduleNumber], isOpenLoop);
    }
}    

/* Used by SwerveControllerCommand in Auto */
public void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Constants.Swerve.maxSpeed);
    
    for(SwerveModule mod : mSwerveMods){
        mod.setDesiredState(desiredStates[mod.moduleNumber], false);
    }
}    

public Pose2d getPose() {
    return swerveOdometry.getPoseMeters();
}

public void resetOdometry(Pose2d pose) {
    swerveOdometry.resetPosition(pose, getYaw());
}

public SwerveModuleState[] getStates(){
    SwerveModuleState[] states = new SwerveModuleState[4];
    for(SwerveModule mod : mSwerveMods){
        states[mod.moduleNumber] = mod.getState();
    }
    return states;
}

public void zeroGyro(){
    gyro.setYaw(0);
}

public Rotation2d getYaw() {
    double[] ypr = new double[3];
    gyro.getYawPitchRoll(ypr);
    return (Constants.Swerve.invertGyro) ? Rotation2d.fromDegrees(360 - ypr[0]) : Rotation2d.fromDegrees(ypr[0]); //TODO set constant
}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    swerveOdometry.update(getYaw(), getStates());  

        for(SwerveModule mod : mSwerveMods){
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Cancoder", mod.getCanCoder().getDegrees());
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Integrated", mod.getState().angle.getDegrees());
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Velocity", mod.getState().speedMetersPerSecond);    
        }
  }
}
