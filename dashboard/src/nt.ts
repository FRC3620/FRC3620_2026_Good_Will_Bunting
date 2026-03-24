import { NetworkTables, NetworkTablesTypeInfos } from "ntcore-ts-client";

const nt = NetworkTables.getInstanceByTeam(3620);

// create topic
const helloTopic = nt.createTopic<string>(
  "/dashboard/hello",
  NetworkTablesTypeInfos.kString,
  "default"
);

let publisherReady = false;

export async function initNT() {
  if (!publisherReady) {
    await helloTopic.publish();
    publisherReady = true;
  }
}

export function sendHello(value: string) {
  if (publisherReady) {
    helloTopic.setValue(value);
  }
}