import { TOPIC_CONFIGS } from "./topics";
import TextWidget from "./widgets/TextWidget";
import TrajectoryWidget from "./widgets/TrajectoryWidget";
import MultiTargetFieldWidget from "./widgets/MultiTargetFieldWidget";
import { type TopicConfig } from "./topics";

function SingleWidget({ topic }: { topic: TopicConfig }) {
    switch (topic.widget) {
        case "text": return <TextWidget topic={topic} />;
        case "trajectory": return <TrajectoryWidget topic={topic} />;
        default: return <div>Unknown widget type: {topic.widget}</div>;
    }
}

export default function Dashboard() {
    const fieldTopics = TOPIC_CONFIGS.filter(t => t.widget === "field");
    const trajectoryTopics = TOPIC_CONFIGS.filter(t => t.widget === "trajectory");
    const otherTopics = TOPIC_CONFIGS.filter(t => t.widget !== "field" && t.widget !== "trajectory");

    return (
        <div style={{
            display: "grid",
            gridTemplateColumns: "1fr 1fr",
            gap: "1.5rem",
            alignItems: "start",
        }}>
            <div>
                {fieldTopics.length > 0 && <MultiTargetFieldWidget topics={fieldTopics} />}
            </div>
            <div style={{ display: "flex", flexDirection: "column", gap: "1rem" }}>
                {trajectoryTopics.length > 0 && <TrajectoryWidget topics={trajectoryTopics} />}
                {otherTopics.map(topic => <SingleWidget key={topic.key} topic={topic} />)}
            </div>
        </div>
    );
}
