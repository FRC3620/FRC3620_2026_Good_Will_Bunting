package org.usfirst.frc3620.logger.tinylog;

import java.io.IOException;
import java.util.Collections;
import java.util.Map;

import org.tinylog.core.LogEntry;
import org.tinylog.writers.AbstractFormatPatternWriter;

import dev.doglog.DogLog;

/**
 * Writer for outputting log entries to a .wpilog file.
 */
public final class DogLogWriter extends AbstractFormatPatternWriter {

  /**
   */
  public DogLogWriter() {
    this(Collections.<String, String>emptyMap());
  }

  /**
   * @param properties
   * Configuration for writer
   */
  public DogLogWriter(final Map<String, String> properties) {
    super(properties);
  }

  @Override
  public void write(final LogEntry logEntry) throws IOException {
    DogLog.log("tinylog", render(logEntry).trim());
  }

  @Override
  public void flush() throws IOException {
  }

  @Override
  public void close() throws IOException {
  }

}
