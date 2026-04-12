package frc.robot.systems.LEDs;

import edu.wpi.first.wpilibj.util.Color;

public class ledConstants {
    
    public static final class HSVLEDColor {
        public static final int red = 90;
        public static final int orange = 100;
        public static final int yellow = 70;
        public static final int green = 30;
        public static final int blue = 0;
        public static final int indigo = 120;
        public static final int purple = 140;
    }

    public enum RGBLEDColor {
        RED(new int[] {240,0,0}),
        BLUE(new int[] {25,25,150});

        int[] arr;
        private RGBLEDColor(int[] pArr) {
            this.arr = pArr;
        }

        public int[] getRGBLEDArray() {
            return arr;
        } 
    }

   }
