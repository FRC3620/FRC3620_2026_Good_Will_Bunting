package frc.robot.Helpers;

import com.ctre.phoenix6.Orchestra;
import com.ctre.phoenix6.configs.AudioConfigs;
import com.ctre.phoenix6.hardware.TalonFX;

public class OrchestraManager {

    private final Orchestra m_orchestra = new Orchestra();
    private boolean m_isLoaded = false;

    /**
     * Pass in any TalonFX motors from your existing subsystems.
     * Each motor plays one track from the .chrp file in order.
     */
    public OrchestraManager(TalonFX... motors) {

    }

    public enum OrchestraSong {
        TITANIUM("Titanium", "titanium.chrp"),
        CALLMEMAYBE("Call me Maybe", "CallMeMaybe.chrp"),
        SANDSTORM("Sandstorm", "Sandstorm.chrp" ),
        IGOTAFEELING("I Got a Feeling", "IGOTAFEELING.chrp"),
        PIRATE("Pirates of the Caribbean", "Pirate.chrp");

        public final String displayName;
        public final String filename;

        OrchestraSong(String displayName, String filename) {
            this.displayName = displayName;
            this.filename = filename;
        }
    }

    public void loadSong(OrchestraSong song) {
        if (song != null) {
            loadSong(song.filename);
        }
    }


    public void addInstrument(TalonFX motor, int track) {
        if (motor == null)
            return;
        AudioConfigs audioConfigs = new AudioConfigs()
                .withAllowMusicDurDisable(true)
                .withBeepOnBoot(false)
                .withBeepOnConfig(false);
        motor.getConfigurator().apply(audioConfigs);
        m_orchestra.addInstrument(motor, track);
    }

    

    /**
     * Load a .chrp file from src/main/deploy.
     * Call this once before trying to play — not periodically.
     */
    public void loadSong(String filename) {
        var status = m_orchestra.loadMusic(filename);
        if (status.isOK()) {
            m_isLoaded = true;
        } else {
            m_isLoaded = false;
        }
    }

    /** Start or resume playback. Does nothing if no song is loaded. */
    public void play() {
        if (m_isLoaded) {
            m_orchestra.play();
        } else {
        }
    }

    /** Pause playback — motors can resume normal control. */
    public void pause() {
        m_orchestra.pause();
    }

    /**
     * Stop playback and reset to beginning.
     * Always call this when the robot enables so motors are free.
     */
    public void stop() {
        m_orchestra.stop();
    }

    /** Returns true if the orchestra is currently playing. */
    public boolean isPlaying() {
        return m_orchestra.isPlaying();
    }

    /**
     * Call this in robotInit() or whenever you want to swap songs.
     * Safe to call multiple times.
     */
    public void reload(String filename) {
        m_orchestra.stop();
        loadSong(filename);
    }
}
