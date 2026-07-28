package frc.robot.Subsystems.Vision;

import edu.wpi.first.networktables.ConnectionInfo;
import edu.wpi.first.networktables.NetworkTableInstance;

import java.io.File;
import java.util.concurrent.Executors;
import java.util.concurrent.ScheduledExecutorService;
import java.util.concurrent.TimeUnit;

public class QuestNavADBWatcher {

    private static final String ADB_PORT = "5802"; // ADB TCP port used by QuestNav

    private static final String ADB_PATH = "/home/lvuser/adb"; // where the ADB executable is located on the rio

    private static final double ADB_TRIGGER_DELAY_S = 0.3; // time before trying to fix with ADB

    private static final double ADB_COOLDOWN_S = 2.0; // time between ADB fix attempts

    private static final int ADB_MAX_RETRIES = 5; // number of times to try the ADB fix

    private static final int POLL_INTERVAL_MS = 300; // intervals to check for pause

    private final ScheduledExecutorService executor;

    private double passthroughStartTime = -1.0;
    private double passthroughLastFixTime = -1.0;
    private int passthroughRetries = 0;

    private volatile boolean questNavHealthy = true;

    // Discovered dynamically from NetworkTables instead of hardcoded.
    // null/empty until a connection whose remote_id looks like "quest" is seen.
    private volatile String questAdbAddress = "";

    // Tracks whether we've already run the one-time TCP/IP + connect handshake
    // for the CURRENT discovered address, so we don't repeat it every poll.
    private String lastHandshakeAddress = "";

    public QuestNavADBWatcher() {
        executor = Executors.newSingleThreadScheduledExecutor(r -> {
            Thread t = new Thread(r, "QuestNavADBWatcher");
            t.setDaemon(true);
            return t;
        });
    }

    public void start() {
        executor.scheduleAtFixedRate(this::poll, 0, POLL_INTERVAL_MS, TimeUnit.MILLISECONDS);
    }

    public void stop() {
        executor.shutdown();
    }

    /**
     * Scans NetworkTables for a connection that looks like the Quest (its
     * remote_id contains "quest"), and records its IP. This mirrors
     * QuestWaker.updateQuestIP() -- it works because the Quest is already
     * talking to NetworkTables (that's how QuestNav pose data gets to the
     * robot in the first place), so no manual IP configuration is needed.
     */
    private void updateQuestIP() {
        ConnectionInfo[] connections = NetworkTableInstance.getDefault().getConnections();
        for (ConnectionInfo conn : connections) {
            if (conn.remote_id.toLowerCase().contains("quest")) {
                String ip = conn.remote_ip.split(":")[0];
                questAdbAddress = ip + ":" + ADB_PORT;
                return;
            }
        }
        // Not found this pass -- leave questAdbAddress as whatever it was
        // (empty if we've never found it, or the last-known address if the
        // Quest's NT connection is just briefly absent).
    }

    private void openPort(String address) {
        try {
            File adbFile = new File(ADB_PATH);
            if (!adbFile.exists()) {
                System.err.println("[QuestNavADBWatcher] adb not found at: " + ADB_PATH
                        + " — update ADB_PATH in QuestNavADBWatcher.java");
                return;
            }

            Process p = new ProcessBuilder(ADB_PATH, "tcpip", ADB_PORT)
                    .redirectErrorStream(true)
                    .start();
            p.waitFor(3, TimeUnit.SECONDS);

        } catch (Exception e) {
            System.err.println("[QuestNavADBWatcher] ADB tcpip command failed: " + e.getMessage());
        }
    }

    private void connectADB(String address) {
        try {
            File adbFile = new File(ADB_PATH);
            if (!adbFile.exists()) {
                System.err.println("[QuestNavADBWatcher] adb not found at: " + ADB_PATH
                        + " — update ADB_PATH in QuestNavADBWatcher.java");
                return;
            }

            Process p = new ProcessBuilder(ADB_PATH, "connect", address)
                    .redirectErrorStream(true)
                    .start();
            p.waitFor(3, TimeUnit.SECONDS);

        } catch (Exception e) {
            System.err.println("[QuestNavADBWatcher] ADB connect failed: " + e.getMessage());
        }
    }

    private void poll() {
        try {
            updateQuestIP();

            String address = questAdbAddress;
            if (address.isEmpty()) {
                // Haven't discovered the Quest's IP yet -- nothing to do.
                return;
            }

            // Run the one-time TCP/IP + connect handshake the first time we see
            // this particular address (covers both "just discovered it for the
            // first time" and "the Quest's IP changed since last time").
            if (!address.equals(lastHandshakeAddress)) {
                openPort(address);
                connectADB(address);
                lastHandshakeAddress = address;
            }

            boolean isInPassthrough = !questNavHealthy;
            double now = nowSeconds();

            if (isInPassthrough) {

                if (passthroughStartTime < 0) {
                    passthroughStartTime = now;
                }

                double elapsedSinceTrigger = now - passthroughStartTime;
                double elapsedSinceLastFix = (passthroughLastFixTime < 0) ? Double.MAX_VALUE
                        : (now - passthroughLastFixTime);

                if (elapsedSinceTrigger > ADB_TRIGGER_DELAY_S) {
                    if (elapsedSinceLastFix > ADB_COOLDOWN_S) {
                        if (passthroughRetries < ADB_MAX_RETRIES) {

                            passthroughRetries++;

                            fireADBRelaunch(address);

                            passthroughLastFixTime = now;

                            passthroughStartTime = now;

                        }
                    }
                }

            } else {

                if (passthroughStartTime >= 0) {

                    passthroughStartTime = -1.0;

                    passthroughLastFixTime = -1.0;

                    passthroughRetries = 0;

                }
            }

        } catch (Exception e) {
            System.err.println("[QuestNavADBWatcher] Unexpected error in poll(): " + e.getMessage());
        }
    }

    private void fireADBRelaunch(String address) {
        try {
            File adbFile = new File(ADB_PATH);
            if (!adbFile.exists()) {
                System.err.println("[QuestNavADBWatcher] adb not found at: " + ADB_PATH);
                return;
            }

            new ProcessBuilder(ADB_PATH, "-s", address,
                    "shell", "am", "start",
                    "-n", "gg.QuestNav.QuestNav/com.unity3d.player.UnityPlayerGameActivity")
                    .redirectErrorStream(true)
                    .start();

        } catch (Exception e) {
            System.err.println("[QuestNavADBWatcher] Failed to execute ADB command: " + e.getMessage());
        }
    }

    private static double nowSeconds() {
        return System.nanoTime() / 1e9;
    }

    public void reportQuestNavHealth(boolean healthy) {
        this.questNavHealthy = healthy;
    }

    /**
     * Exposed for dashboard/debugging so you can see what address was discovered.
     */
    public String getDiscoveredAddress() {
        return questAdbAddress;
    }
}