package frc.robot.util;

import java.util.Arrays;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;

public class MathUtils {
    public static Pose3d calcAvg(Pose3d... values){
        if(values.length == 0) return new Pose3d();
        double[] trlXVals = new double[values.length];
        double[] trlYVals = new double[values.length];
        double[] trlZVals = new double[values.length];
        double[] rotXCosVals = new double[values.length];
        double[] rotXSinVals = new double[values.length];
        double[] rotYCosVals = new double[values.length];
        double[] rotYSinVals = new double[values.length];
        double[] rotZCosVals = new double[values.length];
        double[] rotZSinVals = new double[values.length];
        for(int i = 0; i < values.length; i++) {
            var v = values[i];
            trlXVals[i] = v.getX();
            trlYVals[i] = v.getY();
            trlZVals[i] = v.getZ();
            rotXCosVals[i] = Math.cos(v.getRotation().getX());
            rotXSinVals[i] = Math.sin(v.getRotation().getX());
            rotYCosVals[i] = Math.cos(v.getRotation().getY());
            rotYSinVals[i] = Math.sin(v.getRotation().getY());
            rotZCosVals[i] = Math.cos(v.getRotation().getZ());
            rotZSinVals[i] = Math.sin(v.getRotation().getZ());
        }
        return new Pose3d(
            calcAvg(trlXVals),
            calcAvg(trlYVals),
            calcAvg(trlZVals),
            new Rotation3d(
                new Rotation2d(calcAvg(rotXCosVals), calcAvg(rotXSinVals)).getRadians(),
                new Rotation2d(calcAvg(rotYCosVals), calcAvg(rotYSinVals)).getRadians(),
                new Rotation2d(calcAvg(rotZCosVals), calcAvg(rotZSinVals)).getRadians()
            )
        );
    }
    public static double calcAvg(double... values){
        if(values.length == 0) return 0;
        return Arrays.stream(values).average().getAsDouble();
    }

    public static Pose3d calcStdDev(Pose3d avgPose, Pose3d... values){
        if(values.length == 0) return new Pose3d();

        double[] trlXVals = new double[values.length];
        double[] trlYVals = new double[values.length];
        double[] trlZVals = new double[values.length];
        double rotXDev = 0;
        double rotYDev = 0;
        double rotZDev = 0;
        for(int i = 0; i < values.length; i++){
            var v = values[i];
            trlXVals[i] = v.getX();
            trlYVals[i] = v.getY();
            trlZVals[i] = v.getZ();
            rotXDev += Math.pow(
                MathUtil.angleModulus(v.getRotation().getX() - avgPose.getRotation().getX()),
                2
            );
            rotYDev += Math.pow(
                MathUtil.angleModulus(v.getRotation().getY() - avgPose.getRotation().getY()),
                2
            );
            rotZDev += Math.pow(
                MathUtil.angleModulus(v.getRotation().getZ() - avgPose.getRotation().getZ()),
                2
            );
        }
        rotXDev /= values.length;
        rotXDev = Math.sqrt(rotXDev);
        rotYDev /= values.length;
        rotYDev = Math.sqrt(rotYDev);
        rotZDev /= values.length;
        rotZDev = Math.sqrt(rotZDev);

        return new Pose3d(
            calcStdDev(avgPose.getX(), trlXVals),
            calcStdDev(avgPose.getY(), trlYVals),
            calcStdDev(avgPose.getZ(), trlZVals),
            new Rotation3d(rotXDev, rotYDev, rotZDev)
        );
    }
    public static double calcStdDev(double mean, double... values){
        if(values.length == 0) return 0;
        double variance = 0;
        for(double v : values){
            variance += Math.pow((v - mean), 2);
        }
        variance /= values.length;
        return Math.sqrt(variance);
    }

    public static boolean within(int value, int from, int to){
        return from <= value && value <= to;
    }
    public static boolean within(double value, double from, double to){
        return from <= value && value <= to;
    }
    public static int clamp(int value, int from, int to){
        return Math.max(from, Math.min(value, to));
    }
    public static double clamp(double value, double from, double to){
        return Math.max(from, Math.min(value, to));
    }

    /**
     * Linear percentage from start to end
     * @return Percentage (NOT CLAMPED)
     */
    public static double percentTo(double value, double start, double end) {
        return (value-start)/(end-start);
    }
    /**
     * Linearly interpolates from start to end
     * @return Resulting value (NOT CLAMPED)
     */
    public static double lerp(double percent, double start, double end){
        return start+(end-start)*percent;
    }
    /**
     * Linearly maps value in [startFrom, startTo] to [endFrom, endTo]
     * @return Resulting value (NOT CLAMPED)
     */
    public static double map(double value, double inStart, double inEnd, double outStart, double outEnd) {
        return lerp(percentTo(value, inStart, inEnd), outStart, outEnd);
    }
}
