import { NetworkTables, NetworkTablesTopic, NetworkTablesTypeInfos } from "ntcore-ts-client";
import { type TopicType, TOPIC_CONFIGS, typeInfoMap } from "./topics";

let nt: NetworkTables | null = null;

export type NTValue = string | number | boolean | number[];

const topicRegistry = new Map<string, NetworkTablesTopic<NTValue>>();

let publisherReady = false;

export function connectToRobot(team: number) {
  nt = NetworkTables.getInstanceByTeam(team);
}

export function connectToSimulator(host = "localhost", port = 5810) {
  nt = NetworkTables.getInstanceByURI(host, port);
}

export async function initNT(timeoutMs = 5000): Promise<void> {
  if (!nt) throw new Error("Call connectToRobot or connectToSimulator first");

  const connect = async () => {
    for (const config of TOPIC_CONFIGS) {
      const topic = nt!.createTopic(
        config.key,
        typeInfoMap[config.type],
        config.defaultValue
      );
      await topic.publish();
      topicRegistry.set(config.key, topic as NetworkTablesTopic<NTValue>);
    }
    publisherReady = true;
  };

  const timeout = new Promise<never>((_, reject) =>
    setTimeout(() => reject(new Error("Connection timed out")), timeoutMs)
  );

  try {
    await Promise.race([connect(), timeout]);
  } catch (e) {
    // Reset so the user can retry cleanly
    publisherReady = false;
    nt = null;
    topicRegistry.clear();
    throw e;
  }
}

export function setValue(key: string, value: NTValue) {
  if (!publisherReady) return;
  topicRegistry.get(key)?.setValue(value);
}

export function subscribeToValue(
  key: string, 
  callback: (value: NTValue | null) => void
) {
  topicRegistry.get(key)?.subscribe(callback);
}