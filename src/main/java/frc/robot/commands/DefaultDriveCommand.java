package frc.robot.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.RobotContainer;
import frc.robot.Constants;
import java.util.function.DoubleSupplier;

public class DefaultDriveCommand extends CommandBase {
    private final DrivetrainSubsystem m_drivetrainSubsystem;

    private final DoubleSupplier m_translationXSupplier;
    private final DoubleSupplier m_translationYSupplier;
    private final DoubleSupplier m_rotationSupplier;
    private Double Setpoint;
    private Double RotationSpeed;
    private Double kp = 0.155;
    public DefaultDriveCommand(DrivetrainSubsystem drivetrainSubsystem,
                               DoubleSupplier translationXSupplier,
                               DoubleSupplier translationYSupplier,
                               DoubleSupplier rotationSupplier) {
        this.m_drivetrainSubsystem = drivetrainSubsystem;
        this.m_translationXSupplier = translationXSupplier;
        this.m_translationYSupplier = translationYSupplier;
        this.m_rotationSupplier = rotationSupplier;

        addRequirements(drivetrainSubsystem);
    }

    @Override
    public void execute() {

        if (RobotContainer.driverJoyStick.getLeftBumper()){
            RobotContainer.speedmultiplier = Constants.BOOST_SPEED_MULT;
        }else
        if (RobotContainer.driverJoyStick.getLeftBumper()){
            RobotContainer.speedmultiplier = Constants.SLOW_SPEED_MULT;
        }else RobotContainer.speedmultiplier = Constants.BASE_SPEED_MULT;

        // You can use `new ChassisSpeeds(...)` for robot-oriented movement instead of field-oriented movement
        if (RobotContainer.driverJoyStick.getYButton()||RobotContainer.driverJoyStick.getXButton()||RobotContainer.driverJoyStick.getAButton()||RobotContainer.driverJoyStick.getBButton()){
            if(RobotContainer.driverJoyStick.getAButton()){
                Setpoint = 180.0;
            }else
            if(RobotContainer.driverJoyStick.getBButton()){
                Setpoint = 90.0;
            }else
            if(RobotContainer.driverJoyStick.getYButton()){
                Setpoint = 0.0;
            }else
            if(RobotContainer.driverJoyStick.getXButton()){
                Setpoint = -90.0;
            }
            if((m_drivetrainSubsystem.getAngleDegrees()-Setpoint)>180){
                Setpoint=Setpoint+360;
            }
            if((m_drivetrainSubsystem.getAngleDegrees()-Setpoint)<-180){
                Setpoint=Setpoint-360;
            }
            RotationSpeed = (m_drivetrainSubsystem.getAngleDegrees()-Setpoint)*kp;
            m_drivetrainSubsystem.drive(
                ChassisSpeeds.fromFieldRelativeSpeeds(
                        m_translationXSupplier.getAsDouble()*RobotContainer.speedmultiplier,
                        m_translationYSupplier.getAsDouble()*RobotContainer.speedmultiplier,
                        RotationSpeed*RobotContainer.speedmultiplier,
                        m_drivetrainSubsystem.getGyroscopeRotation()
                )
            );
        }else{

    
        m_drivetrainSubsystem.drive(
                ChassisSpeeds.fromFieldRelativeSpeeds(
                        m_translationXSupplier.getAsDouble()*RobotContainer.speedmultiplier,
                        m_translationYSupplier.getAsDouble()*RobotContainer.speedmultiplier,
                        m_rotationSupplier.getAsDouble()*RobotContainer.speedmultiplier,
                        m_drivetrainSubsystem.getGyroscopeRotation()
                )
        );}
    }

    @Override
    public void end(boolean interrupted) {
        m_drivetrainSubsystem.drive(new ChassisSpeeds(0.0, 0.0, 0.0));
    }
}
