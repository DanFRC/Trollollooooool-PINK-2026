package frc.robot.subsystems.shooter;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class TheStickSubsystem extends SubsystemBase {

    //SerialPort theStick = new SerialPort(9600, SerialPort.Port.kUSB1);

    public TheStickSubsystem() {}


    public void dotheThing(String number) {
        //theStick.writeString(number);
    }

    @Override
    public void periodic() {
        //dotheThing("6");
    }

    

}