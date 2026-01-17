package frc.lib.ml;

import edu.wpi.first.math.geometry.Pose2d;

public class NearestNeighbour {
    
    public NearestNeighbour(){}

    public static double[] getDensities(Pose2d[] poses, double radius){
        Pose2d initialPose;
        Pose2d toPose;
        double[] densities = new double[poses.length];

        for(int i = 0; i < poses.length; i++){
            initialPose = poses[i];
            int count = 0;

            for(int j = 0; j < poses.length; j++){
                
                if(j==i){
                    continue;
                }
                
                toPose = poses[j];
                double distance = computeDensity(initialPose, toPose);
                
                if(distance < radius){
                    count++;
                }
            }
            densities[i] = count;
        }
        return densities;
    }

    private static double computeDensity(Pose2d initial, Pose2d to){
        return Math.sqrt(
            Math.pow((to.getX() - initial.getX()), 2) +
            Math.pow((to.getY() - initial.getY()), 2));
    }
}
