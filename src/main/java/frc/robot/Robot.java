package frc.robot;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private RobotContainer m_robotContainer;

  Joystick driverController = new Joystick(0);
  GenericHID functionController = new GenericHID(1);

  double autoStart = 0;
  


  @Override
  public void robotInit() {

    CameraServer.startAutomaticCapture();
    
    m_robotContainer = new RobotContainer();
  }

  @Override
  public void robotPeriodic() {

    CommandScheduler.getInstance().run();

  }

  @Override
  public void teleopInit() {

    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  @Override
  public void teleopPeriodic() {}

  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    autoStart = Timer.getFPGATimestamp();

    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  @Override
  public void autonomousPeriodic() {
    double autoTimeElapsed = Timer.getFPGATimestamp() - autoStart;
      
      Double autoCode = 1.0;
     
    if(autoCode == 1) {

      if(autoTimeElapsed<1.0) {}

      else {}

    } 
  }

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

   @Override
  public void testPeriodic() {}

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}
}


