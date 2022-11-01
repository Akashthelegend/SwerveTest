package frc.robot;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;

import frc.lib.math.Conversions;
import frc.lib.util.CTREModuleState;
import frc.lib.util.SwerveModuleConstants;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.sensors.CANCoder;

public class SwerveModule {
    
    public int moduleNumber;
    private WPI_TalonFX driveMotor;
    private WPI_TalonFX angleMotor;
    private double angleOffset;
    private CANCoder angleEncoder;
    private double lastAngle;   
    //private final CANCoder driveEncoder; Team364 didn't use a driving encoder as of right now  
    
    private PIDController heading; // TODO We can use the PID controller built into the falcon

    public SwerveModule(int moduleNumber, SwerveModuleConstants moduleConstants){
        this.moduleNumber = moduleNumber;
        angleOffset = moduleConstants.angleOffset;

        angleEncoder = new CANCoder(moduleConstants.cancoderID);
        configAngleEncoder(); //Method will be made later

        angleMotor = new WPI_TalonFX(moduleConstants.angleMotorID);
        configAngleMotor(); //Method will be made later
        
        driveMotor = new WPI_TalonFX(moduleConstants.driveMotorID);
        configDriveMotor(); //Method will be made later

        lastAngle = getState().angle.getDegrees();
    }

public void setDesiredState(SwerveModuleState desiredState, boolean isOpenLoop){
    desiredState = CTREModuleState.optimize(desiredState, getState().angle);

    if (isOpenLoop) {
        double percentOutput = desiredState.speedMetersPerSecond / Constants.Swerve.maxSpeed; //TODO set max speed in constants
            driveMotor.set(ControlMode.PercentOutput, percentOutput);
        }
        else {
            double velocity = Conversions.MPSToFalcon(desiredState.speedMetersPerSecond, Constants.Swerve.wheelCircumference, Constants.Swerve.driveGearRatio); //TODO Set constants
            driveMotor.set(ControlMode.Velocity, velocity, DemandType.ArbitraryFeedForward, feedforward.calculate(desiredState.speedMetersPerSecond)); //TODO set feedforward
        }

        double angle = (Math.abs(desiredState.speedMetersPerSecond) <= (Constants.Swerve.maxSpeed * 0.01)) ? lastAngle : desiredState.angle.getDegrees(); //Deadband to prevent jittering
        angleMotor.set(ControlMode.Position, Conversions.degreesToFalcon(angle, Constants.Swerve.angleGearRatio)); //TODO set constants
        lastAngle = angle;
}

private void resetToAbsolute(){
    double absolutePosition = Conversions.degreesToFalcon(getCanCoder().getDegrees() - angleOffset, Constants.Swerve.angleGearRatio);
    angleMotor.setSelectedSensorPosition(absolutePosition);
}

private void configAngleEncoder(){        
    angleEncoder.configFactoryDefault();
    angleEncoder.configAllSettings(Robot.ctreConfigs.swerveCanCoderConfig); //in constants
}

private void configAngleMotor(){
    angleMotor.configFactoryDefault();
    angleMotor.configAllSettings(Robot.ctreConfigs.swerveAngleFXConfig);
    angleMotor.setInverted(Constants.Swerve.angleMotorInvert);
    angleMotor.setNeutralMode(Constants.Swerve.angleNeutralMode);
    resetToAbsolute();
}

private void configDriveMotor(){        
    driveMotor.configFactoryDefault();
    driveMotor.configAllSettings(Robot.ctreConfigs.swerveDriveFXConfig);
    driveMotor.setInverted(Constants.Swerve.driveMotorInvert);
    driveMotor.setNeutralMode(Constants.Swerve.driveNeutralMode);
    driveMotor.setSelectedSensorPosition(0);
}

public Rotation2d getCanCoder(){
    return Rotation2d.fromDegrees(angleEncoder.getAbsolutePosition());
}

public SwerveModuleState getState(){
    double velocity = Conversions.falconToMPS(driveMotor.getSelectedSensorVelocity(), Constants.Swerve.wheelCircumference, Constants.Swerve.driveGearRatio);
    Rotation2d angle = Rotation2d.fromDegrees(Conversions.falconToDegrees(angleMotor.getSelectedSensorPosition(), Constants.Swerve.angleGearRatio));
    return new SwerveModuleState(velocity, angle);
}

}

