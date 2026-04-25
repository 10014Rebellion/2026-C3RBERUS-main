package frc.robot.systems.music;

public class MusicConstants {
    
    public enum music {
        CANTINA("cantina_band.chrp");

        String mMusicName;
        private music(String pMusicName) {
            this.mMusicName = pMusicName;
        }

        public String getMusicName() {
            return mMusicName;
        }

    }
    public static final String musicName = "cantina_band.chrp";

}
