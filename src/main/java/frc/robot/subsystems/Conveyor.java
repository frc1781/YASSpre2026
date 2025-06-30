package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import java.util.function.BooleanSupplier;
import CRA.CRASparkMax;

public class Conveyor extends SubsystemBase {
    CRASparkMax motor;

    public Conveyor() {
        motor = new CRASparkMax(Constants.Conveyor.MOTOR_CAN_ID, false, false);
    }
    
    public Command clearCoral(BooleanSupplier hasCoralToClear) {
        return new RunCommand(() -> {motor.set(hasCoralToClear.getAsBoolean() ? 0.5 : 0);}, this);
    }         
}