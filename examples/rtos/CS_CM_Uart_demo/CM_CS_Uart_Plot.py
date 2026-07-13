import re
import serial
import matplotlib.pyplot as plt
import matplotlib as mpl

from matplotlib.animation import FuncAnimation
from collections import deque


# ==========================
# matplotlib style
# ==========================

plt.style.use("default")

mpl.rcParams["font.size"] = 11
mpl.rcParams["axes.titleweight"] = "bold"
mpl.rcParams["axes.labelweight"] = "bold"



# ==========================
# Serial config
# ==========================

PORTS = [
    "COM10",
    "COM13",
]

BAUDRATE = 921600



# ==========================
# Display
# ==========================

DIST_MAX_POINTS = 200     # Distance / Velocity / Confidence
RSSI_MAX_POINTS = 1000    # CM RSSI
MIN_WINDOW = 20



# ==========================
# Buffer
# ==========================

distance_x = deque(maxlen=DIST_MAX_POINTS)

distances = deque(maxlen=DIST_MAX_POINTS)
velocities = deque(maxlen=DIST_MAX_POINTS)
confidences = deque(maxlen=DIST_MAX_POINTS)

rssi_x = deque(maxlen=RSSI_MAX_POINTS)

central_rssis = deque(maxlen=RSSI_MAX_POINTS)
peri_rssis = deque(maxlen=RSSI_MAX_POINTS)


distance_index = 0
rssi_index = 0



# ==========================
# Regex
# ==========================

dist_vel_pattern = re.compile(
    r"Distance:\s*(-?\d+).*?"
    r"V:\s*(-?\d+).*?"
    r"C:\s*(-?\d+)"
)


cm_pattern = re.compile(
    r"Central\s+RSSI\s*=\s*(-?\d+).*?"
    r"Peri\s+RSSI\s*=\s*(-?\d+).*?"
    r"Central\s+status\s*=\s*(\d+).*?"
    r"Peri\s+status\s*=\s*(\d+)",
    re.IGNORECASE
)



# ==========================
# Serial open
# ==========================

serials = []

for port in PORTS:

    try:

        ser = serial.Serial(
            port,
            BAUDRATE,
            timeout=0
        )

        serials.append(ser)

        print("Open:", port)

    except Exception as e:

        print(
            "Open failed:",
            port,
            e
        )



# ==========================
# Figure 1
# ==========================

fig1, (ax1, ax2, ax3) = plt.subplots(
    3,
    1,
    figsize=(14,10),
    sharex=True
)



line_distance, = ax1.plot(
    [],
    [],
    color="#00BFFF",
    linewidth=2.5,
    marker=".",
    markersize=4,
    label="Distance"
)


line_velocity, = ax2.plot(
    [],
    [],
    color="#FFA500",
    linewidth=2.5,
    marker=".",
    markersize=4,
    label="Velocity"
)


line_confidence, = ax3.plot(
    [],
    [],
    color="#32CD32",
    linewidth=2.5,
    marker=".",
    markersize=4,
    label="Confidence"
)



ax1.set_title("Real-time Distance")
ax1.set_ylabel("Distance")


ax2.set_title("Real-time Velocity")
ax2.set_ylabel("Velocity")
ax2.set_ylim(-100,400)


ax3.set_title("Real-time Confidence")
ax3.set_ylabel("Confidence")
ax3.set_xlabel("Sample")
ax3.set_ylim(0,120)



for ax in [ax1, ax2, ax3]:

    ax.grid(
        True,
        linestyle="--",
        alpha=0.3
    )

    ax.legend()



# ==========================
# Figure 2
# ==========================

fig2, ax4 = plt.subplots(
    figsize=(14,5)
)



line_central, = ax4.plot(
    [],
    [],
    color="#DA70D6",
    linewidth=2.5,
    marker=".",
    markersize=4,
    label="Central RSSI"
)


line_peri, = ax4.plot(
    [],
    [],
    color="#FF6347",
    linewidth=2.5,
    marker=".",
    markersize=4,
    label="Peri RSSI"
)



ax4.set_title(
    "CM Report RSSI"
)

ax4.set_xlabel(
    "Sample"
)

ax4.set_ylabel(
    "RSSI (dBm)"
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



# ==========================
# X axis
# ==========================
def update_xlim(ax, index, max_points):

    if index < MIN_WINDOW:

        ax.set_xlim(
            0,
            MIN_WINDOW
        )

    elif index < max_points:

        ax.set_xlim(
            0,
            index
        )

    else:

        ax.set_xlim(
            index-max_points,
            index
        )

# ==========================
# Update
# ==========================

def update(frame):

    global distance_index
    global rssi_index



    for ser in serials:


        while ser.in_waiting:


            try:

                line = ser.readline().decode(
                    "utf-8",
                    errors="ignore"
                ).strip()


                if not line:
                    continue



                # ======================
                # Distance
                # ======================

                m = dist_vel_pattern.search(line)


                if m:

                    print(line)


                    distance = int(m.group(1))
                    velocity = int(m.group(2))
                    confidence = int(m.group(3))


                    distance_x.append(
                        distance_index
                    )

                    distances.append(
                        distance
                    )

                    velocities.append(
                        velocity
                    )

                    confidences.append(
                        confidence
                    )


                    distance_index += 1




                # ======================
                # CM RSSI
                # ======================

                m = cm_pattern.search(line)


                if m:


                    # print(line)


                    central = int(m.group(1))
                    peri = int(m.group(2))


                    central_status = int(m.group(3))
                    peri_status = int(m.group(4))


                    rssi_x.append(
                        rssi_index
                    )


                    if central_status != 0 and central_status != 3:

                        central_rssis.append(
                            central
                        )

                    else:

                        central_rssis.append(
                            None
                        )



                    if peri_status != 0 and peri_status != 3:

                        peri_rssis.append(
                            peri
                        )

                    else:

                        peri_rssis.append(
                            None
                        )


                    rssi_index += 1



            except Exception as e:

                print(
                    "Parse error:",
                    e
                )



    # ======================
    # Update Figure 1
    # ======================

    line_distance.set_data(
        distance_x,
        distances
    )


    line_velocity.set_data(
        distance_x,
        velocities
    )


    line_confidence.set_data(
        distance_x,
        confidences
    )


    update_xlim(
        ax1,
        distance_index,
        DIST_MAX_POINTS
    )

    update_xlim(
        ax2,
        distance_index,
        DIST_MAX_POINTS
    )   

    update_xlim(
        ax3,
        distance_index,
        DIST_MAX_POINTS
    )



    if len(distances):

        ax1.set_ylim(
            min(distances)-100,
            max(distances)+100
        )



    # ======================
    # Update Figure 2
    # ======================

    line_central.set_data(
        rssi_x,
        central_rssis
    )


    line_peri.set_data(
        rssi_x,
        peri_rssis
    )


    update_xlim(
        ax4,
        rssi_index,
        RSSI_MAX_POINTS
    )

    # 强制刷新第二窗口

    fig2.canvas.draw_idle()



    return (
        line_distance,
        line_velocity,
        line_confidence,
        line_central,
        line_peri
    )



# ==========================
# Animation
# ==========================

ani = FuncAnimation(
    fig1,
    update,
    interval=50,
    blit=False,
    cache_frame_data=False
)



plt.tight_layout(
    pad=2,
    h_pad=2
)



plt.show()



# ==========================
# Close
# ==========================

for ser in serials:

    ser.close()