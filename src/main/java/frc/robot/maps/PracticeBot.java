package frc.robot.maps;

import com.chopshop166.chopshoplib.outputs.SendableSpeedController;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.interfaces.Potentiometer;
import frc.robot.RobotMap;
import frc.robot.subsystems.Lift;

public class PracticeBot implements RobotMap {

    @Override
    public ManipulatorMap getManipulatorMap() {
        return new ManipulatorMap() {

            @Override
            public SendableSpeedController getrollersMotor() {
                return SendableSpeedController.wrap(new WPI_TalonSRX(2));
            }

            @Override
            public SendableSpeedController getpivotPointsMotor() {
                return SendableSpeedController.wrap(new WPI_TalonSRX(3));
            }

            @Override
            public DigitalInput getintakePositionLimitSwitch() {
                return new DigitalInput(1);
            }

            @Override
            public DigitalInput getfoldedBackLimitSwitch() {
                return new DigitalInput(2);
            }

            @Override
            public DoubleSolenoid getbeaksPiston() {
                return new DoubleSolenoid(2, 3);
            }

            @Override
            public DigitalInput getGamepieceLimitSwitch() {
                return new DigitalInput(3);
            }
        };
    }
    // Fill in any methods from the interface here.

    @Override
    public LiftMap getLiftMap() {
        return new LiftMap() {

            @Override
            public SendableSpeedController getLiftMotor() {
                return SendableSpeedController.wrap(new WPI_VictorSPX(7));
            }

            @Override
            public Encoder getHeightEncoder() {
                return new Encoder(2, 5);
            }

            @Override
            public DigitalInput getTopLimitSwitch() {
                return new DigitalInput(3);
            }

            @Override
            public DigitalInput getBottomLimitSwitch() {
                return new DigitalInput(3);
            }

        };
    }

    @Override
    public MaflipulatorMap getMaflipulatorMap() {
        return new MaflipulatorMap() {

            @Override
            public Potentiometer getManipulatorPot() {
                return new AnalogPotentiometer(8);
            }

            @Override
            public SendableSpeedController getFlipMotor() {
                return SendableSpeedController.wrap(new WPI_VictorSPX(4));
            }
        };
    }
}