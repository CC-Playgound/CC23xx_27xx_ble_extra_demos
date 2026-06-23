import re
import matplotlib.pyplot as plt
from pathlib import Path

log_dir = Path(__file__).parent
pattern = re.compile(r"Central Connection RSSI (-?\d+)")
data = {}  # dist -> list of RSSI values

for dist in range(1, 11):
    log_file = log_dir / f"{dist}m_ce.log"
    if not log_file.exists():
        print(f"Skipping {log_file.name} (not found)")
        continue

    rssi_values = []
    with open(log_file, "r") as f:
        for line in f:
            m = pattern.search(line)
            if m:
                value = int(m.group(1))
                if value != -127:
                    rssi_values.append(value)

    if rssi_values:
        data[dist] = rssi_values
        print(f"{dist}m: {len(rssi_values)} samples, avg={sum(rssi_values)/len(rssi_values):.1f} dBm")

# --- Figure 1: RSSI over sample number ---
fig1, ax1 = plt.subplots(figsize=(14, 6))
for dist, rssi_values in data.items():
    ax1.plot(rssi_values, label=f"{dist}m", linewidth=0.8)
ax1.set_xlabel("Sample Number")
ax1.set_ylabel("Central RSSI (dBm)")
ax1.set_title("Central RSSI vs Sample Number")
ax1.legend(title="Distance", loc="lower right")
ax1.grid(True, alpha=0.3)
fig1.tight_layout()
fig1.savefig(log_dir / "central_connection_rssi_timeseries.png", dpi=150)
print("Saved to central_rssi_timeseries.png")

# --- Figure 2: All RSSI values grouped by distance ---
fig2, ax2 = plt.subplots(figsize=(12, 6))
labels = [f"{d}m" for d in data.keys()]
box_data = list(data.values())

bp = ax2.boxplot(box_data, labels=labels, patch_artist=True, showfliers=True,
                 flierprops=dict(marker=".", markersize=2, alpha=0.4))

colors = plt.cm.tab20.colors
for patch, color in zip(bp["boxes"], colors):
    patch.set_facecolor(color)
    patch.set_alpha(0.6)

ax2.set_xlabel("Distance")
ax2.set_ylabel("Central RSSI (dBm)")
ax2.set_title("Central RSSI Distribution by Distance")
ax2.grid(True, axis="y", alpha=0.3)
fig2.tight_layout()
fig2.savefig(log_dir / "central_connection_rssi_by_distance.png", dpi=150)
print("Saved to central_rssi_by_distance.png")

# --- Figure 3: Scatter plot of all samples by distance ---
fig3, ax3 = plt.subplots(figsize=(12, 6))
colors = plt.cm.tab20.colors
for i, (dist, rssi_values) in enumerate(data.items()):
    x = [dist] * len(rssi_values)
    ax3.scatter(x, rssi_values, s=4, alpha=0.3, color=colors[i % len(colors)], label=f"{dist}m")

ax3.set_xticks(list(data.keys()))
ax3.set_xticklabels([f"{d}m" for d in data.keys()])
ax3.set_xlabel("Distance")
ax3.set_ylabel("Central RSSI (dBm)")
ax3.set_title("CE RSSI All Samples by Distance")
ax3.legend(title="Distance", loc="lower right", markerscale=3)
ax3.grid(True, axis="y", alpha=0.3)
fig3.tight_layout()
fig3.savefig(log_dir / "central_connection_rssi_scatter.png", dpi=150)
print("Saved to central_rssi_scatter.png")

plt.show()
