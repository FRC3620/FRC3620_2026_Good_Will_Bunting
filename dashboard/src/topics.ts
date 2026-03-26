import { NetworkTablesTopic, NetworkTablesTypeInfos } from "ntcore-ts-client";

// Defines what types a topic can carry
export type TopicType = "string" | "number" | "boolean" | "number[]";

// A single topic definition
export interface TopicConfig {
  key: string;           // NT path e.g. "/dashboard/speed"
  type: TopicType;
  defaultValue: string | number | boolean | number[];
  label: string;         // Human readable name for the UI
  widget: WidgetType;    // What kind of widget to render
}

export type WidgetType = "text" | "gauge" | "toggle" | "indicator" | "field";

// Map our simple type names to ntcore's type system
export const typeInfoMap = {
  string: NetworkTablesTypeInfos.kString,
  number: NetworkTablesTypeInfos.kDouble,
  boolean: NetworkTablesTypeInfos.kBoolean,
  "number[]": NetworkTablesTypeInfos.kDoubleArray
};

// ✏️ Add or remove topics here to change your dashboard
export const TOPIC_CONFIGS: TopicConfig[] = [
  {
    key: "/dashboard/hello",
    type: "string",
    defaultValue: "",
    label: "Hello Message",
    widget: "text",
  },
 {
    key: "/dashboard/target/hub",
    type: "number[]",
    defaultValue: [4.627, 4.034, 0.584], // BLUE_HUB converted to meters
    label: "Blue Hub",
    widget: "field",
  },
  {
    key: "/dashboard/target/op_pass",
    type: "number[]",
    defaultValue: [3.2, 2.511, 0],
    label: "Op Pass",
    widget: "field",
  },
  {
    key: "/dashboard/target/depot_pass",
    type: "number[]",
    defaultValue: [3.2, 5.559, 0],
    label: "Depot Pass",
    widget: "field",
  },
];