package org.usfirst.frc.team3042.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.buttons.Button;
import edu.wpi.first.wpilibj.buttons.JoystickButton;

/**
 * This class is the glue that binds the controls on the physical operator
 * interface to the commands and command groups that allow control of the robot.
 */
public class OI {
    //// CREATING BUTTONS
    // One type of button is a joystick button which is any button on a joystick.
    // You create one by telling it which joystick it's on and which button
    // number it is.
    // Joystick stick = new Joystick(port);
    // Button button = new JoystickButton(stick, buttonNumber);
    
    // There are a few additional built in buttons you can use. Additionally,
    // by subclassing Button you can create custom triggers and bind those to
    // commands the same as any other Button.
    
    //// TRIGGERING COMMANDS WITH BUTTONS
    // Once you have a button, it's trivial to bind it to a button in one of
    // three ways:
    
    // Start the command when the button is pressed and let it run the command
    // until it is finished as determined by it's isFinished method.
    // button.whenPressed(new ExampleCommand());
    
    // Run the command while the button is being held down and interrupt it once
    // the button is released.
    // button.whileHeld(new ExampleCommand());
    
    // Start the command when the button is released  and let it run the command
    // until it is finished as determined by it's isFinished method.
    // button.whenReleased(new ExampleCommand());
	//Declare the joysticks
	public Joystick joystickLeft = new Joystick(RobotMap.LEFT_JOYSTICK_USB_PORT_0);
	public Joystick joystickRight = new Joystick(RobotMap.RIGHT_JOYSTICK_USB_PORT_1);
	
	//Left Joystick Buttons
	public Button left_1 = new JoystickButton(joystickLeft, 1);
	public Button left_3 = new JoystickButton(joystickLeft, 3);
	public Button left_4 = new JoystickButton(joystickLeft, 4);
	public Button left_5 = new JoystickButton(joystickLeft, 5);
	public Button left_6 = new JoystickButton(joystickLeft, 6);
	public Button left_8 = new JoystickButton(joystickLeft, 8);

	//Right Joystick Buttons
	public Button right_1 = new JoystickButton(joystickRight, 1);
	public Button right_2 = new JoystickButton(joystickRight, 2);
	public Button right_3 = new JoystickButton(joystickRight, 3);
	public Button right_4 = new JoystickButton(joystickRight, 4);
	public Button right_5 = new JoystickButton(joystickRight, 5);
}

