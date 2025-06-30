package CRA;

import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

public class CRASparkMax extends SparkMax {


    
    public CRASparkMax(int canid, boolean inverted, boolean idleBreak, int stallLimit) {
        super(canid, SparkLowLevel.MotorType.kBrushless);
        SparkMaxConfig config = new SparkMaxConfig();
        config.idleMode(idleBreak ? SparkBaseConfig.IdleMode.kBrake :  SparkBaseConfig.IdleMode.kCoast);
        config.smartCurrentLimit(stallLimit);
        config.inverted(inverted);
        super.configure(config, 
            SparkBase.ResetMode.kResetSafeParameters, 
            SparkBase.PersistMode.kPersistParameters
        );
    }

    public CRASparkMax(int canid, boolean inverted, boolean idleBreak) {
        this(canid, inverted, idleBreak, 30);
    }
}
