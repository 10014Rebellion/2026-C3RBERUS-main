package frc.lib.sorting;

import edu.wpi.first.math.geometry.Pose2d;

public class Pose2dSorter {
    
    /**
     * Returns a sorted copy of the array without modifying the original.
     * Uses QuickSort based on translation norm.
     * 
     * @param poses The array of Pose2d objects to sort
     * @return A new sorted array
     */
    public static Pose2d[] quickSort(Pose2d[] poses) {
        if (poses == null || poses.length == 0) {
            return poses;
        }
        Pose2d[] copy = poses.clone();
        quickSort(copy, 0, copy.length - 1);
        return copy;
    }
    
    /**
     * Recursive QuickSort helper method.
     * 
     * @param poses The array of Pose2d objects
     * @param low The starting index
     * @param high The ending index
     */
    private static void quickSort(Pose2d[] poses, int low, int high) {
        if (low < high) {
            int partitionIndex = partition(poses, low, high);
            quickSort(poses, low, partitionIndex - 1);
            quickSort(poses, partitionIndex + 1, high);
        }
    }
    
    /**
     * Partitions the array around a pivot element.
     * 
     * @param poses The array of Pose2d objects
     * @param low The starting index
     * @param high The ending index
     * @return The partition index
     */
    private static int partition(Pose2d[] poses, int low, int high) {
        double pivot = poses[high].getTranslation().getNorm();
        int i = low - 1;
        
        for (int j = low; j < high; j++) {
            if (poses[j].getTranslation().getNorm() < pivot) {
                i++;
                swap(poses, i, j);
            }
        }
        
        swap(poses, i + 1, high);
        return i + 1;
    }
    
    /**
     * Swaps two elements in the array.
     * 
     * @param poses The array of Pose2d objects
     * @param i Index of first element
     * @param j Index of second element
     */
    private static void swap(Pose2d[] poses, int i, int j) {
        Pose2d temp = poses[i];
        poses[i] = poses[j];
        poses[j] = temp;
    }
}