import { TOPIC_CONFIGS } from "./topics";
import TextWidget from "./widgets/TextWidget";
import MultiTargetFieldWidget from "./widgets/MultiTargetFieldWidget";
import { type TopicConfig } from "./topics";

function SingleWidget({ topic }: { topic: TopicConfig }) {
  switch (topic.widget) {
    case "text":      return <TextWidget topic={topic} />;
    default:          return <div>Unknown widget type: {topic.widget}</div>;
  }
}

export default function Dashboard() {
  const fieldTopics = TOPIC_CONFIGS.filter(t => t.widget === "field");
  const otherTopics = TOPIC_CONFIGS.filter(t => t.widget !== "field");

  return (
    <div>
      {/* All field targets share one combined widget */}
      {fieldTopics.length > 0 && (
        <MultiTargetFieldWidget topics={fieldTopics} />
      )}

      {/* Everything else renders individually */}
      <div style={{ display: "grid", gridTemplateColumns: "repeat(3, 1fr)", gap: "1rem" }}>
        {otherTopics.map(topic => (
          <SingleWidget key={topic.key} topic={topic} />
        ))}
      </div>
    </div>
  );
}