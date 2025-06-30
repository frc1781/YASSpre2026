package CRA;

public class Util {

    public static double clamp(double min, double max, double value) {
        if (value > max) {
            return max;
        }
        else if (value < min) {
            return min;
        }
        else {
            return value;
        }

    }
    
}
