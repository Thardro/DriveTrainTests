package org.usfirst.frc.team3042.robot.commands;

import org.usfirst.frc.team3042.robot.Robot;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class TankDriveTrain_ArcadeDrive extends Command {

	//Scale the joystick values to restrict maximum speed
    private final double speedScale = 1.0;
    
    private final double deadzone = 0.07;
    
    Timer timer = new Timer();
    //Inertial dampening
    final int LEFT = 0, RIGHT = 1;
    double[] oldTime = new double[] {0, 0};
    double[] currentPower = new double[] {0,0};
    double maxAccel = 3.6; //motor power per second
	
    public TankDriveTrain_ArcadeDrive() {
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
    	requires(Robot.tankDriveTrain);
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    	Robot.logger.log("Initialize", 1);
    	
    	Robot.tankDriveTrain.setMotors(0, 0);
    	timer.start();
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    	double rotation = Robot.oi.joystickLeft.getX() * speedScale;
        double power = -Robot.oi.joystickRight.getY() * speedScale;
        
        rotation = (Math.abs(rotation) < deadzone)? 0 : rotation;
        power = (Math.abs(power) < deadzone)? 0 : power;
        
        double leftPower = power + rotation;
        double rightPower = power - rotation;
        
        leftPower = restrictAccel(leftPower, LEFT);
    	rightPower = restrictAccel(rightPower, RIGHT);
        
        Robot.tankDriveTrain.setMotors(leftPower, rightPower);
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
        return false;
    }

    // Called once after isFinished returns true
    protected void end() {
    	Robot.logger.log("End", 1);
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    	Robot.logger.log("Interrupt", 1);
    }
    
    private double restrictAccel (double goalValue, int TYPE) {
        double currentTime = timer.get();
        double dt = currentTime - oldTime[TYPE];
        oldTime[TYPE] = currentTime;
        
        double maxDSpeed = maxAccel * dt;
        maxDSpeed *= (goalValue >= currentPower[TYPE])? 1 : -1;
         
        currentPower[TYPE] = (Math.abs(maxDSpeed) > Math.abs(goalValue - currentPower[TYPE]))? 
                goalValue : maxDSpeed + currentPower[TYPE];
        return currentPower[TYPE];
    }
}
