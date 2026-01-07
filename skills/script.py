import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
from PIL import Image

# Load lemlib data
data = pd.read_csv(
    "real_skills_data.txt",
    header=None,
    names=["x", "y", "theta"]
)

# Load field image (240x240 px)
field_img = Image.open("field.png")

FIELD_IN = 144  # VEX field size in inches

plt.figure(figsize=(6, 6))

# Draw field
plt.imshow(
    field_img,
    extent=[-FIELD_IN/2, FIELD_IN/2, -FIELD_IN/2, FIELD_IN/2]
)

# Plot path
plt.plot(data["x"], data["y"], "r", linewidth=2)
plt.scatter(data["x"], data["y"], s=4, c="r")

# Heading arrows (lemlib theta is degrees)
theta = np.deg2rad(data["theta"])
plt.quiver(
    data["x"][::10],
    data["y"][::10],
    np.cos(theta[::10]),
    np.sin(theta[::10]),
    scale=25,
    width=0.004,
    color="blue"
)

plt.axis("equal")
plt.xlabel("X (in)")
plt.ylabel("Y (in)")
plt.title("Lemlib Odometry Overlay")
plt.show()
