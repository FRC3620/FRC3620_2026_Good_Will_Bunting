package org.usfirst.frc3620.logger;

import org.tinylog.TaggedLogger;

import edu.wpi.first.wpilibj2.command.InstantCommand;

@SuppressWarnings("unused")
public class LogCommand extends InstantCommand {
  static TaggedLogger defaultLogger = LoggingMaster.getLogger(LogCommand.class);

  private final TaggedLogger logger;

  private final String msg;

  private final Object[] args;

  public LogCommand(String message) {
    this(null, message, null);
  }

  public LogCommand(String message, Object[] args) {
    this(null, message, args);
  }

  public LogCommand(TaggedLogger constructorLogger, String message) {
    this(constructorLogger, message, null);
  }

  public LogCommand(TaggedLogger constructorLogger, String message, Object[] args) {
    logger = (constructorLogger == null) ? defaultLogger : constructorLogger;
    msg = message;
    this.args = args;
  }

  @Override
  public void initialize() {
    if (args == null) {
      logger.info(msg);
    } else {
      logger.info(msg, args);
    }
  }

  @Override
  public boolean runsWhenDisabled() {
    return true;
  }
}
