package org.usfirst.frc3620;

import com.fasterxml.jackson.databind.JavaType;
import com.fasterxml.jackson.databind.ObjectMapper;
import com.fasterxml.jackson.databind.type.TypeFactory;

import frc.robot.Robot;

import org.tinylog.TaggedLogger;
import org.usfirst.frc3620.logger.LoggingMaster;

import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotController;

import java.io.File;
import java.io.IOException;
import java.lang.reflect.Constructor;
import java.lang.reflect.InvocationTargetException;
import java.lang.reflect.Modifier;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.util.*;

public class RobotParametersContainer {
    public final static TaggedLogger logger = LoggingMaster.getLogger(RobotParametersBase.class);

    static ObjectMapper objectMapper = new ObjectMapper();

    public static <T extends RobotParametersBase> Map<String, T> makeParameterMap(List<T> l) {
        Map<String, T> rv = new HashMap<>();
        for (T c : l) {
            for (var serialNumber : c.serialNumbers) {
                rv.put(serialNumber.toLowerCase(), c);
            }
        }
        return rv;
    }

    static <T extends RobotParametersBase> Map<String, T> readConfiguration(Class<T> parametersClass, Path path) throws IOException {
        String json = Files.readString(path);
        json = Minifier.minify(json);
        // https://stackoverflow.com/a/61154659/17887564
        JavaType javaType = TypeFactory.defaultInstance().constructCollectionType(List.class, parametersClass);
        List<T> list = objectMapper.readValue(json, javaType);

        return makeParameterMap(list);
    }

    public static <T extends RobotParametersBase> T getRobotParameters (Class<T> parametersClass, String filename) {
        Path path = Paths.get(filename);
        var rv = getRobotParameters(parametersClass, path, identifyRoboRIO());
        if (rv == null) {
            rv = getRobotParameters(parametersClass, path, "(unknown)");
        }
        return rv;
    }

    public static <T extends RobotParametersBase> T getRobotParameters (Class<T> parametersClass) {
        File file = new File(Filesystem.getDeployDirectory(), "robot_parameters.json");
        return getRobotParameters(parametersClass, file.toPath(), identifyRoboRIO());
    }

    @SuppressWarnings("unchecked")
    public static <T extends RobotParametersBase> T getRobotParameters (Class<T> parametersClass, Path path, String serialNumber) {
        if (! RobotParametersBase.class.isAssignableFrom(parametersClass)) {
            logger.error("getRobotParameters needs a subclass of RobotParameters, returning null");
            return null;
        }
        RobotParametersBase rv = null;

        Map<String, T> parameterMap = null;
        logger.info("reading {} from {}", parametersClass.getName(), path);
        try {
            parameterMap = readConfiguration(parametersClass, path);
        } catch (IOException e) {
            logger.error ("can't read configuration at {}", path);
            logger.error ("caused by {}", e);
        }

        if (parameterMap != null) {
            rv = parameterMap.get(serialNumber.toLowerCase());
            if (rv == null) {
                logger.info ("no entry in {} for \"{}\"", path, serialNumber);
            }
        }
        if (rv == null) {
            if (parametersClass.isMemberClass() && ((parametersClass.getModifiers() & Modifier.STATIC) == 0)) {
                logger.error("Cannot make a default value; this is a non-static inner class");
            } else {
                try {
                    logger.info("making default {}", parametersClass.getName());
                    Constructor<? extends RobotParametersBase> c = parametersClass.getConstructor();
                    rv = c.newInstance();
                } catch (InvocationTargetException | InstantiationException | IllegalAccessException | NoSuchMethodException e) {
                    e.printStackTrace(System.err);
                    logger.error("got exception, returning null RobotParameters", e);
                }
            }
        }
        if (rv != null) {
            logger.info("robot parameters {}: {}", rv.getClass(), rv);
            /*
            try {
                // looks like Jackson is not serializing all fields?
                logger.info("robot parameters {}: {}", rv.getClass(), objectMapper.writeValueAsString(rv));
            } catch (JsonProcessingException ex) {
                logger.error("Unable to format JSON of RobotParameters for logging: {}", ex);
            }
            */
        }
        return (T) rv;
    }

    static String roboRIOSerialNumber = null;

    public static String identifyRoboRIO() {
        if (roboRIOSerialNumber == null) {
            roboRIOSerialNumber = "(simulation)";
            if (Robot.isReal()) {
                roboRIOSerialNumber = RobotController.getSerialNumber();;
            }
            logger.info ("Serial number = '{}'", roboRIOSerialNumber);
        }
        return roboRIOSerialNumber;
    }

}
