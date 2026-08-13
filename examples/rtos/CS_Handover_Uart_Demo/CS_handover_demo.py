import serial
import threading
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import re


# =========================
# UART Config
# =========================

PORT1 = "COM13"
PORT2 = "COM47"

BAUDRATE = 921600


# =========================
# Data
# =========================

distance1 = []
distance2 = []

lock = threading.Lock()


# =========================
# Regex
# =========================

pattern = re.compile(
    r"Distance:\s*(\d+)"
)


# =========================
# UART Reader
# =========================

def uart_reader(port, data, name):

    try:
        ser = serial.Serial(
            port,
            BAUDRATE,
            timeout=1
        )

        print(
            f"{name} opened {port}"
        )

    except Exception as e:

        print(
            f"{name} open failed:",
            e
        )

        return


    while True:

        try:

            line = ser.readline().decode(
                errors="ignore"
            ).strip()


            if not line:
                continue


            match = pattern.search(line)


            if match:

                distance = int(
                    match.group(1)
                )


                with lock:

                    data.append(distance)


                print(
                    f"{name}: Distance={distance} cm"
                )


        except Exception as e:

            print(
                name,
                e
            )

            break



# =========================
# Start UART threads
# =========================

threading.Thread(
    target=uart_reader,
    args=(
        PORT1,
        distance1,
        "COM13"
    ),
    daemon=True
).start()


threading.Thread(
    target=uart_reader,
    args=(
        PORT2,
        distance2,
        "COM47"
    ),
    daemon=True
).start()



# =========================
# Plot Setup
# =========================

fig, axes = plt.subplots(
    2,
    1,
    figsize=(10, 7)
)


line1, = axes[0].plot(
    [],
    []
)

line2, = axes[1].plot(
    [],
    []
)


axes[0].set_title(
    "COM13 Distance"
)

axes[1].set_title(
    "COM47 Distance"
)


for ax in axes:

    ax.grid(True)

    ax.set_xlabel(
        "Sample"
    )

    ax.set_ylabel(
        "Distance (cm)"
    )



# =========================
# Auto Y axis
# =========================

def auto_scale_y(ax, data):

    if len(data) < 2:
        return


    ymin = min(data)
    ymax = max(data)


    # 防止只有一个值时范围为0
    if ymin == ymax:

        margin = 20

    else:

        margin = (ymax - ymin) * 0.2

        # 最小margin
        margin = max(
            margin,
            10
        )


    ax.set_ylim(
        ymin - margin,
        ymax + margin
    )



# =========================
# Update
# =========================

def update(frame):

    with lock:

        d1 = distance1.copy()
        d2 = distance2.copy()


    # COM13

    if len(d1):

        line1.set_data(
            range(len(d1)),
            d1
        )

        axes[0].set_xlim(
            0,
            max(
                50,
                len(d1)
            )
        )

        auto_scale_y(
            axes[0],
            d1
        )


    # COM47

    if len(d2):

        line2.set_data(
            range(len(d2)),
            d2
        )

        axes[1].set_xlim(
            0,
            max(
                50,
                len(d2)
            )
        )

        auto_scale_y(
            axes[1],
            d2
        )


    return (
        line1,
        line2
    )



ani = FuncAnimation(
    fig,
    update,
    interval=200
)


plt.tight_layout()

plt.show()