import { useState, useEffect } from "react";
import { setValue, subscribeToValue } from "../nt";
import { type TopicConfig } from "../topics";

export default function TextWidget({ topic }: { topic: TopicConfig }) {
  const [value, setValue_] = useState(topic.defaultValue as string);
  const [input, setInput] = useState("");

  useEffect(() => {
    subscribeToValue(topic.key, (v) => setValue_(v as string ?? ""));
  }, [topic.key]);

  return (
    <div>
      <label>{topic.label}</label>
      <p>Current: {value}</p>
      <input value={input} onChange={(e) => setInput(e.target.value)} />
      <button onClick={() => { setValue(topic.key, input); setInput(""); }}>
        Send
      </button>
    </div>
  );
}