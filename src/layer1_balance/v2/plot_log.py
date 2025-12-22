import pandas as pd
import matplotlib.pyplot as plt

df = pd.read_csv("run_log.csv")

t = df["t"] - df["t"].iloc[0]

plt.figure(figsize=(12,8))

plt.subplot(3,1,1)
plt.plot(t, df["roll"], label="roll")
plt.plot(t, df["posture_roll"], label="posture_roll")
plt.ylabel("deg")
plt.legend()
plt.grid()

plt.subplot(3,1,2)
plt.plot(t, df["roll_effort"], label="roll_effort")
plt.plot(t, df["offset_change"], label="offset_change")
plt.ylabel("effort")
plt.legend()
plt.grid()

plt.subplot(3,1,3)
plt.plot(t, df["brace"], label="brace")
plt.ylabel("brace")
plt.xlabel("time (s)")
plt.grid()

plt.tight_layout()
plt.show()
