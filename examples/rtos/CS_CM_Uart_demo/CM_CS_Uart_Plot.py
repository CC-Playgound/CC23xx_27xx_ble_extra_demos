import re
import csv
import serial
import matplotlib.pyplot as plt
import matplotlib as mpl
from datetime import datetime
from collections import deque
from matplotlib.animation import FuncAnimation
# ================= Config =================
PORTS = [
    "COM36",
    "COM13"
]
BAUDRATE = 921600
DIST_MAX = 100
RSSI_MAX = 1000
MIN_WINDOW = 20
# Distance Y axis
AUTO_SCALE_DISTANCE = False
SHOW_VELOCITY = True
DIST_Y_MIN = -100
DIST_Y_MAX = 1000
mpl.rcParams["font.size"] = 11
# ================= CSV =================
csv_file = open(
    f"distance_log_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv",
    "w",
    newline="",
    encoding="utf-8"
)
csv_writer = csv.writer(csv_file)
csv_writer.writerow(
    [
        "Timestamp",
        "Sample",
        "Distance",
        "Velocity",
        "Confidence"
    ]
)
# ================= Buffer =================
distance_x = deque(maxlen=DIST_MAX)
distances = deque(maxlen=DIST_MAX)
velocities = deque(maxlen=DIST_MAX)
confidences = deque(maxlen=DIST_MAX)
rssi_x = deque(maxlen=RSSI_MAX)
central_rssi = deque(maxlen=RSSI_MAX)
peri_rssi = deque(maxlen=RSSI_MAX)
distance_idx = 0
rssi_idx = 0
# ================= Regex =================
dist_pattern = re.compile(
    r"\[(\d+)\].*?"
    r"Distance:\s*(-?\d+).*?"
    r"V:\s*(-?\d+).*?"
    r"C:\s*(-?\d+)"
)
cm_pattern = re.compile(
    r"Central\s+RSSI\s*=\s*(-?\d+).*?"
    r"Peri\s+RSSI\s*=\s*(-?\d+).*?"
    r"Central\s+status\s*=\s*(\d+).*?"
    r"Peri\s+status\s*=\s*(\d+)",
    re.I
)
# ================= Serial =================
serials=[]
for p in PORTS:
    try:
        s = serial.Serial(
            p,
            BAUDRATE,
            timeout=0
        )
        serials.append(s)
        print("Open:",p)
    except Exception as e:
        print(
            "Open failed:",
            p,
            e
        )
# ================= Plot =================
plt.style.use("default")
fig1,(ax1,ax3)=plt.subplots(
    2,
    1,
    figsize=(14,8),
    sharex=True
)
# ================= Control Info =================
control_text = ax1.text(
    0.02,
    0.95,
    "",
    transform=ax1.transAxes,
    fontsize=7,
    verticalalignment="top",
    horizontalalignment="left",
    bbox=dict(
        boxstyle="round",
        facecolor="white",
        alpha=0.6,
        edgecolor="gray",
        pad=0.3
    )
)
# Distance
line_distance, = ax1.plot(
    [],
    [],
    color="#1f77b4",
    linewidth=1.5,
    marker=".",
    markersize=3,
    label="Distance"
)
ax1.set_title(
    "Distance / Velocity"
)
ax1.set_ylabel(
    "Distance (cm)"
)
ax1.grid(
    True,
    linestyle="--",
    alpha=0.3
)
# Velocity
ax2=ax1.twinx()
line_velocity, = ax2.plot(
    [],
    [],
    color="#ffbb78",      # 淡橙色
    linewidth=1.0,        # 更细
    marker=".",
    markersize=2,
    alpha=0.8,            # 透明度
    label="Velocity"
)
ax2.set_ylabel(
    "Velocity (cm/s)"
)
ax2.set_ylim(
    -300,
    300
)
ax1.legend(
    [
        line_distance,
        line_velocity
    ],
    [
        "Distance",
        "Velocity"
    ]
)
# Confidence
line_confidence, = ax3.plot(
    [],
    [],
    color="#2ca02c",
    linewidth=1.5,
    marker=".",
    markersize=3,
    label="Confidence"
)
ax3.set_title(
    "Confidence"
)
ax3.set_ylabel(
    "Confidence"
)
ax3.set_xlabel(
    "Sample"
)
ax3.set_ylim(
    0,
    120
)
ax3.grid(
    True,
    linestyle="--",
    alpha=0.3
)
ax3.legend()
# ================= RSSI =================
fig2,ax4=plt.subplots(
    figsize=(14,5)
)
line_central, = ax4.plot(
    [],
    [],
    color="#9467bd",
    linewidth=1.5,
    marker=".",
    markersize=2.5,
    label="Central RSSI"
)
line_peri, = ax4.plot(
    [],
    [],
    color="#d62728",
    linewidth=1.5,
    marker=".",
    markersize=2.5,
    label="Peri RSSI"
)
ax4.set_title(
    "CM Report RSSI"
)
ax4.set_xlabel(
    "Sample"
)
ax4.set_ylabel(
    "RSSI(dBm)"
)
ax4.set_ylim(
    -128,
    0
)
ax4.grid(
    True,
    linestyle="--",
    alpha=0.3
)
ax4.legend()
# ================= Utils =================
def update_xlim(ax,index,maxp):
    if index < MIN_WINDOW:
        ax.set_xlim(
            0,
            MIN_WINDOW
        )
    elif index < maxp:
        ax.set_xlim(
            0,
            index
        )
    else:
        ax.set_xlim(
            index-maxp,
            index
        )
def update_control_text():
    control_text.set_text(
        "Keyboard Control:\n"
        f"[A] Auto Scale Distance : "
        f"{'ON' if AUTO_SCALE_DISTANCE else 'OFF'}\n"
        f"[V] Velocity Display   : "
        f"{'ON' if SHOW_VELOCITY else 'OFF'}\n"
        "[R] Reset Distance Range\n"
        "Mouse wheel: Zoom"
    )
# ================= Keyboard Control =================
def key_event(event):
    global AUTO_SCALE_DISTANCE
    global SHOW_VELOCITY
    if event.key.lower() == "a":
        AUTO_SCALE_DISTANCE = not AUTO_SCALE_DISTANCE
        print(
            "Auto scale:",
            AUTO_SCALE_DISTANCE
        )
    elif event.key.lower() == "v":
        SHOW_VELOCITY = not SHOW_VELOCITY
        line_velocity.set_visible(
            SHOW_VELOCITY
        )
        ax2.set_visible(
            SHOW_VELOCITY
        )
        print(
            "Velocity:",
            SHOW_VELOCITY
        )
    elif event.key.lower() == "r":
        AUTO_SCALE_DISTANCE = False
        ax1.set_ylim(
            DIST_Y_MIN,
            DIST_Y_MAX
        )
        print(
            "Distance reset"
        )
    update_control_text()
    fig1.canvas.draw_idle()
    
# ================= Update =================
def update(frame):
    global distance_idx
    global rssi_idx
    for ser in serials:
        while ser.in_waiting:
            try:
                line = ser.readline().decode(
                    errors="ignore"
                ).strip()
                if not line:
                    continue
                print(line)
                # Distance
                m = dist_pattern.search(line)
                if m:
                    ts,d,v,c = map(
                        int,
                        m.groups()
                    )
                    distance_x.append(
                        distance_idx
                    )
                    distances.append(d)
                    velocities.append(v)
                    confidences.append(c)
                    csv_writer.writerow(
                        [
                            ts,
                            distance_idx,
                            d,
                            v,
                            c
                        ]
                    )
                    if distance_idx % 100 == 0:
                        csv_file.flush()
                    distance_idx += 1
                # RSSI
                m = cm_pattern.search(line)
                if m:
                    cr,pr,cs,ps = map(
                        int,
                        m.groups()
                    )
                    rssi_x.append(
                        rssi_idx
                    )
                    central_rssi.append(
                        cr if cs not in (0,3)
                        else None
                    )
                    peri_rssi.append(
                        pr if ps not in (0,3)
                        else None
                    )
                    rssi_idx += 1
            except Exception as e:
                print(
                    "Parse error:",
                    e
                )
    # ================= Update Distance =================
    line_distance.set_data(
        distance_x,
        distances
    )
    if SHOW_VELOCITY:
        line_velocity.set_data(
            distance_x,
            velocities
        )
    else:
        line_velocity.set_data(
            [],
            []
        )
    line_confidence.set_data(
        distance_x,
        confidences
    )
    for ax in [
        ax1,
        ax2,
        ax3
    ]:
        update_xlim(
            ax,
            distance_idx,
            DIST_MAX
        )
    if AUTO_SCALE_DISTANCE:
        if distances:
            ax1.set_ylim(
                min(distances)-100,
                max(distances)+100
            )
    else:
        ax1.set_ylim(
            DIST_Y_MIN,
            DIST_Y_MAX
        )
    # ================= Update RSSI =================
    line_central.set_data(
        rssi_x,
        central_rssi
    )
    line_peri.set_data(
        rssi_x,
        peri_rssi
    )
    update_xlim(
        ax4,
        rssi_idx,
        RSSI_MAX
    )
    fig2.canvas.draw_idle()
    return (
        line_distance,
        line_velocity,
        line_confidence,
        line_central,
        line_peri
    )
# ================= Animation =================
update_control_text()
ani = FuncAnimation(
    fig1,
    update,
    interval=50,
    cache_frame_data=False
)
fig1.canvas.mpl_connect(
    "key_press_event",
    key_event
)

fig1.subplots_adjust(
    left=0.055,
    right=0.90,
    top=0.95,
    bottom=0.07,
    hspace=0.18
)


fig2.subplots_adjust(
    left=0.06,
    right=0.96,
    top=0.92,
    bottom=0.12
)
plt.show()
# ================= Close =================
for s in serials:
    s.close()
csv_file.close()