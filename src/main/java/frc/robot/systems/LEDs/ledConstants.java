package frc.robot.systems.LEDs;

import edu.wpi.first.wpilibj.util.Color;

public class ledConstants {
    public static final int ledID = 1;
    public static final int ledBuffer = 13;

    public static final class HSVLEDColor {
        public static final int red = 90;
        public static final int orange = 100;
        public static final int yellow = 70;
        public static final int green = 30;
        public static final int blue = 0;
        public static final int indigo = 120;
        public static final int purple = 140;
    }

    public enum LEDColor {
        RED(Color.kGreen), //The LEDs are GRB ;_;
        GREEN(Color.kRed), 
        BLUE(Color.kBlue),
        ORANGE(Color.kOrange),
        PINK(Color.kPink),
        GRAY(Color.kGray),
        YELLOW(Color.kYellow),
        CYAN(Color.kCyan),
        WHITE(Color.kWhite);

        Color color;
        private LEDColor(Color pColor) {
            this.color = pColor;
        }

        public Color getLEDColor() {
            return color;
        } 

        public static Color[] getLEDColorRandomArr() {
            return new Color[] {
                ORANGE.getLEDColor(),
                PINK.getLEDColor(),
                GRAY.getLEDColor(),
            };
        }
    }

   }
