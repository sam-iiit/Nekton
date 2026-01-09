import React from "react";

export default function GridContainer({ items, children, columns = 3, gap = 12 }) {
  const defaultItems = [
    "Box 1",
    "Box 2",
    "Box 3",
    "Box 4",
    "Box 5",
    "Box 6",
  ];
  const list = items ?? defaultItems;

  const containerStyle = {
    display: "grid",
    gridTemplateColumns: `repeat(${columns}, 1fr)`,
    gap: `${gap}px`,
    alignItems: "center",
    justifyItems: "stretch",
  };

  const boxStyle = {
    background: "#f0f0f0",
    padding: "12px",
    borderRadius: "8px",
    textAlign: "center",
  };

  
  if (children) {
    return (
      <div className="grid-container" style={containerStyle}>
        {children}
      </div>
    );
  }

  const hasCustom = typeof window !== "undefined" && window.customElements && window.customElements.get;

  return (
    <div className="grid-container" style={containerStyle}>
      {list.map((label, i) => {
        const isCustomSlot = i % 2 === 0; // original alternating pattern

        if (isCustomSlot) {
          if (hasCustom && window.customElements.get && window.customElements.get("test-comp")) {
            return React.createElement(
              "test-comp",
              { key: i, text: label }
            );
          }
        }

        return (
          <div key={i} className="box" style={boxStyle}>
            {label}
          </div>
        );
      })}
    </div>
  );
}