package org.usfirst.frc3620;

import java.nio.ByteBuffer;
import java.nio.ByteOrder;
import java.util.*;

import org.tinylog.TaggedLogger;
import org.usfirst.frc3620.logger.LoggingMaster;

import edu.wpi.first.hal.can.CANJNI;

/**
 * Class to find out which goodies are on the CAN bus. The important guts of this
 * were provided by Omar at CTRE; see
 * <a href="https://www.chiefdelphi.com/t/how-to-detect-missing-can-devices-from-java/147675/10">...</a>
 */
@SuppressWarnings("unused")
public class CANDeviceFinder {
    static TaggedLogger logger = LoggingMaster.getLogger(CANDeviceFinder.class);

    Set<CANDeviceId> deviceSet = new TreeSet<>();

    /*
     * this is a map, keyed by CANDeviceType, whose values are sets contained
     * the device numbers for all the present devices of that type.
     */
    Map<CANDeviceType, Set<Integer>> byDeviceType = new TreeMap<>();
    Set<NamedCANDevice> missingDeviceSet = new TreeSet<>();

    public CANDeviceFinder() {
        super();
        find();
        // research();
    }

    public static class NamedCANDevice implements Comparable<NamedCANDevice>{
        CANDeviceId id;
        String name;
        NamedCANDevice (CANDeviceType deviceType, int i, String name) {
            this.id = new CANDeviceId(deviceType, i);
            this.name = name;
        }

        @Override
        public int compareTo(NamedCANDevice namedCANDevice) {
            return id.compareTo(namedCANDevice.id);
        }

        @Override
        public String toString() {
            if (name == null) {
                return id.toString();
            } else {
                return id.toString() + " (" + name + ")";
            }
        }

        public CANDeviceId getCANDeviceId() {
            return id;
        }
    }

    public boolean isDevicePresent(CANDeviceType deviceType, int id) {
        return isDevicePresent(deviceType, id, null);
    }

    public boolean isDevicePresent(CANDeviceType deviceType, int id, String whatItIs) {
        boolean rv = false;
        Set<Integer> deviceTypeSet = byDeviceType.get(deviceType);
        if (deviceTypeSet != null) {
            rv = deviceTypeSet.contains(id);
        }
        if (!rv) {
            NamedCANDevice namedCANDevice = new NamedCANDevice(deviceType, id, whatItIs);
            logger.warn("{} is missing from the CAN bus!!!", namedCANDevice);
            missingDeviceSet.add(namedCANDevice);
        }
        return rv;
    }

    public boolean isPowerDistributionPresent() {
        boolean rv = isDevicePresent(CANDeviceType.REV_PDH, 1);
        if (!rv) rv = isDevicePresent(CANDeviceType.CTRE_PDP, 0);
        return rv;
    }

    /**
     * 
     * @return Set of CANDeviceIDs of all the devices we've found.
     */
    public Set<CANDeviceId> getDeviceSet() {
        return deviceSet;
    }

    public Set<NamedCANDevice> getMissingDeviceSet() { return missingDeviceSet; }

    abstract class CanFinder {
        // constructor should fill this with the IDs of the messages to look for.
        int[] ids;
        long[] ts0;

        // pass2 will fill this with the IDs of messages from 'ids' that actually showed up
        Set<Integer> idsPresent = new TreeSet<>();

        void pass1() {
            ts0 = new long[ids.length];
            for (int i = 0; i < ids.length; ++i) {
                ts0[i] = checkMessage(ids[i]);
                //logger.info ("pass1 looking for {} got {}", String.format("%08x", ids[i]), ts0[i]);
            }
        }

        void pass2() {
            long[] ts1 = new long[ids.length];
            for (int i = 0; i < ids.length; ++i) {
                ts1[i] = checkMessage(ids[i]);
                //logger.info ("pass2 looking for {} got {}", String.format("%08x", ids[i]), ts1[i]);
            }
            for (int i = 0; i < ids.length; ++i) {
                if (ts0[i] >= 0 && ts1[i] >= 0 && ts0[i] != ts1[i]) {
                    // logger.info ("found {}", String.format("%08x", ids[i]));
                    idsPresent.add(ids[i]);
                }
            }
        }

        // override this to process the idsPresent list
        abstract void report();
    }

    class DeviceFinder extends CanFinder {
        Set<CANDeviceId> deviceSet;
        CANDeviceType canDeviceType;
        DeviceFinder(CANDeviceType canDeviceType, Set<CANDeviceId> deviceSet) {
            super();

            this.deviceSet = deviceSet;
            this.canDeviceType = canDeviceType;

            ids = new int[canDeviceType.getMaxDevices()];
            for (int i = 0; i < canDeviceType.getMaxDevices(); i++) {
                ids[i] = (canDeviceType.getMsgId() & 0xffffffc0) | (i & 0x3f);
            }
        }

        @Override
        void report() {
            for (int id: idsPresent) {
                int deviceId = extractDeviceId(id);
                deviceSet.add(new CANDeviceId(canDeviceType, deviceId));
            }
        }
    }

    class APIFinder extends CanFinder {
        CANDeviceType canDeviceType;
        APIFinder(CANDeviceType canDeviceType, int deviceId) {
            super();
            this.canDeviceType = canDeviceType;

            ids = new int[1024];
            for (int i = 0; i < 1024; i++) {
                ids[i] = (canDeviceType.msgId & 0xffff0000) | ((i << 6) & 0xffc0) | (deviceId & 0x3f);
            }
        }

        @Override
        void report() {
            for (int id: idsPresent) {
                int apiId = extractApiId(id);
                logger.info ("{}: API {} {} = msg {}", canDeviceType, String.format("%2d", apiId), String.format("0x%03X", apiId), String.format("%08X", id));
            }
        }
    }
    
    /**
     * polls for received framing to determine if a device is deviceSet. This is
     * meant to be used once initially (and not periodically) since this steals
     * cached messages from the robot API.
     */
    public void find() {
        deviceSet.clear();
        byDeviceType.clear();
        missingDeviceSet.clear();

        logger.debug ("calling find()");
        List<CanFinder> finders = new ArrayList<>();

        for (CANDeviceType canDeviceType : CANDeviceType.values()) {
            finders.add(new DeviceFinder(canDeviceType, deviceSet));
        }

        findDetails(finders);
    }

    public void research() {
        logger.info ("calling research()");
        List<CanFinder> finders = new ArrayList<>();
        
        finders.add(new APIFinder(CANDeviceType.CTRE_PCM, 0)); // PCM 0
        finders.add(new APIFinder(CANDeviceType.CTRE_PDP, 0)); // PDP 0
        finders.add(new APIFinder(CANDeviceType.REV_PH, 1)); // PH 1
        finders.add(new APIFinder(CANDeviceType.REV_PDH, 1)); // PDH 1
        finders.add(new APIFinder(CANDeviceType.TALON_PHOENIX5, 1)); // SRX #1
        finders.add(new APIFinder(CANDeviceType.TALON_PHOENIX6, 21)); // FX #21
        finders.add(new APIFinder(CANDeviceType.VICTOR_SPX, 2)); // SPX #2

        findDetails(finders);
    }

    @SuppressWarnings("CallToPrintStackTrace")
    void findDetails(List<CanFinder> finders) {
        for (CanFinder finder: finders) {
            finder.pass1();
        }

        /* wait 200ms */
        try {
            Thread.sleep(200);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }

        for (CanFinder finder: finders) {
            finder.pass2();
        }

        for (CanFinder finder: finders) {
            finder.report();
        }

        /*
         fill in the byDeviceType map.
         */
        for (CANDeviceId canDeviceId: deviceSet) {
            CANDeviceType canDeviceType = canDeviceId.getDeviceType();
            Set<Integer> deviceNumberSet = byDeviceType.computeIfAbsent(canDeviceType, k -> new TreeSet<>());
            deviceNumberSet.add(canDeviceId.getDeviceNumber());
        }
    }
    
    private int extractDeviceId (int canId) {
        return canId & 0x3f;
    }

    private int extractApiId (int canId) {
        return (canId & 0xffc0) >> 6;
    }

    /** helper routine to get last received message for a given ID */
    private final ByteBuffer targetID = ByteBuffer.allocateDirect(4);
    private final ByteBuffer timeStamp = ByteBuffer.allocateDirect(4);
    private long checkMessage(int id) {
        try {
            targetID.clear();
            targetID.order(ByteOrder.LITTLE_ENDIAN);
            targetID.asIntBuffer().put(0, id);

            timeStamp.clear();
            timeStamp.order(ByteOrder.LITTLE_ENDIAN);
            timeStamp.asIntBuffer().put(0, 0x00000000);

            CANJNI.FRCNetCommCANSessionMuxReceiveMessage(
                    targetID.asIntBuffer(), 0x1fffffff, timeStamp);

            long rv = timeStamp.getInt();
            rv &= 0xFFFFFFFFL; /* undo sign-extension */
            return rv;
        } catch (Exception e) {
            return -1;
        }
    }
}