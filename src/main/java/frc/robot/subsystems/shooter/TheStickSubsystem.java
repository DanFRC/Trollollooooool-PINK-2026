package frc.robot.subsystems.shooter;

import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class TheStickSubsystem extends SubsystemBase {

    SerialPort theStick = new SerialPort(9600, SerialPort.Port.kOnboard);

    public TheStickSubsystem() {
        theStick.writeString("1");
    }


    public void dotheThing(String number) {
        theStick.writeString(number);
    }

    @Override
    public void periodic() {
    }

    

}