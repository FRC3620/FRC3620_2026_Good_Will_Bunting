import { TOPIC_CONFIGS } from "./topics";
import TextWidget from "./widgets/TextWidget";
import MultiTargetFieldWidget from "./widgets/MultiTargetFieldWidget";
import { type TopicConfig } from "./topics";

function SingleWidget({ topic }: { topic: TopicConfig }) {
    switch (topic.widget) {
        case "text": return <TextWidget topic={topic} />;
        default: return <div>Unknown widget type: {topic.widget}</div>;
    }
}

export default function Dashboard() {
    const fieldTopics = TOPIC_CONFIGS.filter(t => t.widget === "field");
    const otherTopics = TOPIC_CONFIGS.filter(t => t.widget !== "field");

    return (
        <div style={{
            display: "grid",
            gridTemplateColumns: "1fr 1fr",
            gap: "1.5rem",
            height: "start", // 80px accounts for the header height
        }}>

            {/* Left column — field */}
            <div style={{ display: "flex", flexDirection: "column", minHeight: 0 }}>
                {fieldTopics.length > 0 && (
                    <MultiTargetFieldWidget topics={fieldTopics} />
                )}
            </div>

            {/* Right column — other widgets + placeholder */}
            <div style={{
                display: "flex",
                flexDirection: "column",
                gap: "1rem",
                minHeight: 0,
                overflowY: "auto",
            }}>
                {otherTopics.map(topic => (
                    <SingleWidget key={topic.key} topic={topic} />
                ))}

                {/* Placeholder panel */}
                <div style={{
                    flex: 1,
                    border: "1px dashed var(--border)",
                    borderRadius: "4px",
                    display: "flex",
                    alignItems: "center",
                    justifyContent: "center",
                    flexDirection: "column",
                    gap: "0.5rem",
                    color: "var(--text-secondary)",
                    fontFamily: "var(--text-mono)",
                    fontSize: "0.75rem",
                    letterSpacing: "0.15em",
                    textTransform: "uppercase",
                }}>
                    <span style={{ fontSize: "1.5rem", opacity: 0.3 }}>⊕</span>
                    Widgets Coming Soon
                </div>
            </div>

        </div>
    );
}
