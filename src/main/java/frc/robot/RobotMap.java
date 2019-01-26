package frc.robot;

import com.chopshop166.chopshoplib.outputs.SendableSpeedController;
import com.chopshop166.chopshoplib.sensors.Lidar;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.interfaces.Gyro;

/**
 * The RobotMap is an interface that contains the instructions on how to make a
 * particular robot. This is useful for situations of practice robot vs real
 * robot, where port numbers might differ.
 */
public interface RobotMap {
    // In the interface, create a function that returns a sensor interface or
    // SendableSpeedController.
    // In the map implementations, fill in that function with the specific instance.
    
    DriveMap getDriveMap();

    public interface DriveMap {
        SendableSpeedController getLeftMotors();

        SendableSpeedController getRightMotors();

        DoubleSolenoid getClimbPiston();
        
        Lidar getLidar();
        
        Encoder getLeftEncoder();

        Encoder getRightEncoder();
        
        Gyro getGyro();
    }
}
