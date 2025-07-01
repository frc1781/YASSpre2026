package CRA;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

public class PIDTuning {
    double kP;
    double kI;
    double kD;
    double kF;
    String mechanism;

    public PIDTuning(String mechanism_name, double starting_kP, double starting_kI, double starting_kD, double starting_kF) {
        this.kP = starting_kP;
        this.kI = starting_kI;
        this.kD = starting_kD;
        this.kF = starting_kF;
        this.mechanism = mechanism_name;

        ShuffleboardTab tab = Shuffleboard.getTab(mechanism);
    }

    public void onDashboard() {
        
    }
}
