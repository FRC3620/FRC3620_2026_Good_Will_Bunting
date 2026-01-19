package org.usfirst.frc3620;

import org.tinylog.TaggedLogger;

import org.usfirst.frc3620.logger.LoggingMaster;

import java.io.File;
import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.nio.file.StandardCopyOption;
import java.util.*;
import java.util.concurrent.ConcurrentLinkedQueue;

public class FileSaver {
    public final static TaggedLogger logger = LoggingMaster.getLogger(FileSaver.class);

    Queue<Path> path_queue = new ConcurrentLinkedQueue<>();
    Set<Path> completed_paths = new HashSet<>();

    static FileSaver fileSaver = new FileSaver();

    static public void add(String filename) {
        Path path = Paths.get(filename);
        path = path.toAbsolutePath().normalize();
        File file = path.toFile();
        if (file.isFile()) {
            logger.info("added {} to queue", path);
            fileSaver.path_queue.add(path);
        } else {
            logger.error("unable to add {} to queue, not a file", path);
        }
    }

    private FileSaver () {
        Timer timer = new Timer();
        // every 1000 ms
        timer.schedule(new FileSaverTask(), 0, 1000);
    }

    class FileSaverTask extends TimerTask {
        Path logging_dir = LoggingMaster.getLoggingDirectory().toPath();
        String prefix = null;

        @Override
        public void run() {
            Date now = LoggingMaster.getTimestamp();
            if (now != null) {
                if (!path_queue.isEmpty()) {
                    if (prefix == null) {
                        prefix = "backup_" + LoggingMaster.convertTimestampToString(now) + "_";
                    }
                    while (true) {
                        Path input_path = path_queue.poll();
                        if (input_path == null) break;

                        logger.info ("got {} from queue", input_path);

                        if (completed_paths.contains(input_path))
                            continue;

                        String just_the_file = prefix + input_path.getFileName().toString();
                        Path output_path = logging_dir.resolve(just_the_file).toAbsolutePath().normalize();

                        try {
                            Files.copy(input_path, output_path, StandardCopyOption.COPY_ATTRIBUTES);
                            logger.info("copied from {} to {}", input_path, output_path);
                        } catch (IOException ex) {
                            logger.error ("trouble copying from {} to {}: {}", input_path, output_path, ex);
                        }
                    }
                }
            }
        }
    }

}
