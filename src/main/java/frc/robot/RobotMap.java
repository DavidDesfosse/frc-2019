package frc.robot;

import com.chopshop166.chopshoplib.outputs.SendableSpeedController;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.interfaces.Potentiometer;

/**
 * The RobotMap is an interface that contains the instructions on how to make a
 * particular robot. This is useful for situations of practice robot vs real
 * robot, where port numbers might differ.
 */
public interface RobotMap {
    // In the interface, create a function that returns a sensor interface or
    // SendableSpeedController.
    // In the map implementations, fill in that function with the specific instance.

    ManipulatorMap getManipulatorMap();

    interface ManipulatorMap {

        SendableSpeedController getpivotPointsMotor();

        SendableSpeedController getrollersMotor();

        DoubleSolenoid getbeaksPiston();

        DigitalInput getGamepieceLimitSwitch();

        DigitalInput getfoldedBackLimitSwitch();

        DigitalInput getintakePositionLimitSwitch();
    }

    LiftMap getLiftMap();

    interface LiftMap {

        SendableSpeedController getLiftMotor();

        Encoder getHeightEncoder();

        DigitalInput getTopLimitSwitch();

        DigitalInput getBottomLimitSwitch();
    }

    MaflipulatorMap getMaflipulatorMap();

    interface MaflipulatorMap {

        SendableSpeedController getFlipMotor();

        Potentiometer getManipulatorPot();
    }
}
