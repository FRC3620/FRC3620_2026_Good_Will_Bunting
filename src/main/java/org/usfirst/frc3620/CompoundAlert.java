package org.usfirst.frc3620;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;

public class CompoundAlert {
  String name;
  String currentText = "";
  AlertType currentAlertType = null;

  final Alert infoAlert, warningAlert, errorAlert;

  public CompoundAlert(String groupname) {
    this(groupname, (String) null);
  }

  public CompoundAlert(String groupname, Class<?> clazz) {
    this(groupname, clazz.getSimpleName());
  }

  public CompoundAlert(String groupname, String name) {
    this.name = name;

    infoAlert = new Alert(groupname, "", AlertType.kInfo);
    warningAlert = new Alert(groupname, "", AlertType.kWarning);
    errorAlert = new Alert(groupname, "", AlertType.kError);

    reset();
  }

  void reset() {
    infoAlert.set(false);
    warningAlert.set(false);
    errorAlert.set(false);

    infoAlert.setText("");
    warningAlert.setText("");
    errorAlert.setText("");
  }

  void setAlert(Alert a, AlertType alertType, String text) {
    var alertTypeText  = alertType == null ? "(null)" : alertType.toString();
    if (currentAlertType != alertType || currentText != text) {
      reset();
      if (text != null && a != null) {
        if (name != null)
          text = name + ": " + text;
        a.setText(text);
        a.set(true);
      }
      currentAlertType = alertType;
      currentText = text;
    }
  }

  public void info(String text) {
    setAlert(infoAlert, AlertType.kInfo, text);
  }

  public void warning(String text) {
    setAlert(warningAlert, AlertType.kWarning, text);
  }

  public void error(String text) {
    setAlert(errorAlert, AlertType.kError, text);
  }

  public void none() {
    setAlert(null, null, null);
  }
}
