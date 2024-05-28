import yaml
from my_functions import *

with open("config.yaml", "r") as file:
    config = yaml.safe_load(file)

root = config["root"]
ros2_path_imu = root + config["ros2_paths"]["imu"]
ros2_path_gps = root + config["ros2_paths"]["gps"]
path_imu_sim = root + config["simulation_data_paths"]["imu_sim"]
path_odo_sim = root + config["simulation_data_paths"]["odo_sim"]
path_gps_sim = root + config["simulation_data_paths"]["gps_sim"]
path_imu_hardware = root + config["hardware_data_paths"]["imu_hardware"]
path_magnetometer_hardware = root + config["hardware_data_paths"]["magnetometer_hardware"]
path_odo_hardware = root + config["hardware_data_paths"]["odo_hardware"]
path_gps_hardware = root + config["hardware_data_paths"]["gps_hardware"]
ahrs_alg = config["ahrs_alg"]
plot_data = config["plot_data"]

if root == "/path/to/your/project":
    raise Exception("YOU HAVE TO CHANGE ROOT DIRECTORY IN CONFIG.YAML!")
elif root[-1] == "/":
    raise Exception("BAD PATH FORMATTING LAST SYMBOL CANNOT BE '/'")

# Nastavení parametrů letu a načtení dat z CSV souborů
flight_length = 50  # Délka letu v sekundách
to_s = 1000000  # Konverzní konstanta z mikrosekund na sekundy a obráceně

skip_time = 20  # Čas v sekundách, od kterého se začnou data brát v úvahu
skip_time *= to_s

# Načtení GPS dat z CSV souboru
gps = pd.read_csv(path_gps_hardware)
max_time = max(gps["timestamp"])  # Maximální časový údaj v logu
min_time = min(gps["timestamp"])  # Minimální časový údaj v logu
min_val = min_time
real_length = min_time + flight_length * to_s  # Výpočet skutečné délky letu v mikrosekundách
max_val = max_time if real_length >= max_time else real_length  # Nastavení maximální hodnoty času pro filtraci dat

# Filtrace dat podle času
gps = gps[(gps["timestamp"] < max_val)]
gps = gps[["timestamp", "lat", "lon", "alt"]]  # Výběr relevantních sloupců
gps.reset_index(drop=True, inplace=True)  # Reset indexů DataFrame
gps["timestamp"] = gps["timestamp"] - gps.loc[0, "timestamp"]  # Normalizace časových údajů

# Načtení dat změřené magnetometrem do proměnných
mag = pd.read_csv(path_magnetometer_hardware)
mag = mag[(mag["timestamp"] < max_val)]
mag.reset_index(drop=True, inplace=True)
mag["timestamp"] = mag["timestamp"] - mag.loc[0, "timestamp"]
mx_ = mag["magnetometer_ga[0]"]
my_ = mag["magnetometer_ga[1]"]
mz_ = mag["magnetometer_ga[2]"]

# Načtení IMU dat z CSV souboru
imu_log = pd.read_csv(path_imu_hardware)
imu_log = imu_log[(imu_log["timestamp"] < max_val)]
imu_log.reset_index(drop=True, inplace=True)
imu_log["timestamp"] = imu_log["timestamp"] - imu_log.loc[0, "timestamp"]
imu_time = imu_log["timestamp"]

# Interpolace dat z magnetometru na časovou osu IMU
mx_interp = np.interp(imu_time, mag["timestamp"], mx_)
my_interp = np.interp(imu_time, mag["timestamp"], my_)
mz_interp = np.interp(imu_time, mag["timestamp"], mz_)

# Výpočet časových rozdílů pro jednotlivé vzorky
delta_t = pd.Series(imu_log["gyro_integral_dt"] / 1000000, name="delta_t")
board_estimator = pd.read_csv(path_odo_hardware)
board_estimator = board_estimator[(board_estimator["timestamp"] < max_val)]
board_estimator.reset_index(drop=True, inplace=True)
board_estimator["timestamp"] = board_estimator["timestamp"] - board_estimator.loc[0, "timestamp"]
board_time = board_estimator["timestamp"]

# Extrakce dat z IMU
ax = pd.Series(imu_log["accelerometer_m_s2[0]"], name="x")
ay = pd.Series(imu_log["accelerometer_m_s2[1]"], name="y")
az = pd.Series(imu_log["accelerometer_m_s2[2]"], name="z")
gx = pd.Series(imu_log["gyro_rad[0]"], name="x")
gy = pd.Series(imu_log["gyro_rad[1]"], name="y")
gz = pd.Series(imu_log["gyro_rad[2]"], name="z")
mx = pd.Series(mx_interp, name="x")
my = pd.Series(my_interp, name="y")
mz = pd.Series(mz_interp, name="z")

# Inicializace vektorů pro odhadované úhly
roll_estimated = np.zeros(len(board_estimator))
pitch_estimated = np.zeros(len(board_estimator))
yaw_estimated = np.zeros(len(board_estimator))

# Výpočet eulerových úhlů z quaternionů určené UAV algoritmem (jedná se o referenci)
# Úhly lze využít pro estimaci trajektorie, ale kvůli nesplnění podmínek estimace polohy pouze z IMU, nepoužívame
for i in range(len(board_estimator)):
    [r, p, y] = ahrs.common.orientation.q2rpy(
        np.array([board_estimator.loc[i, "q[0]"], board_estimator.loc[i, "q[1]"], board_estimator.loc[i, "q[2]"],
                  board_estimator.loc[i, "q[3]"]]))
    roll_estimated[i] = r * 180 / np.pi
    pitch_estimated[i] = p * 180 / np.pi
    yaw_estimated[i] = y * 180 / np.pi

# Vytvoření DataFrame pro akcelerometr, gyroskop a magnetometr
accelerometer = pd.DataFrame({"t": imu_time, "ax": ax, "ay": ay, "az": az})
gyroscope = pd.DataFrame({"t": imu_time, "gx": gx, "gy": gy, "gz": gz})
magnetometer = pd.DataFrame({"t": imu_time, "mx": mx, "my": my, "mz": mz})

# Filtrace dat pomocí exponenciálního vyrovnávacího filtru
tau = 0.4  # Filtrační konstanta (1.0 = bez filtrace, 0.0 =  přefiltrování)
ax_filter = exp_filter(ax, tau)
ay_filter = exp_filter(ay, tau)
az_filter = exp_filter(az, tau)
gx_filter = exp_filter(gx, tau)
gy_filter = exp_filter(gy, tau)
gz_filter = exp_filter(gz, tau)
mx_filter = exp_filter(mx, tau)
my_filter = exp_filter(my, tau)
mz_filter = exp_filter(mz, tau)

# Vytvoření DataFrame pro filtrovaná data z IMU
accelerometer_filter = pd.DataFrame({"t": imu_time, "ax": ax_filter, "ay": ay_filter, "az": az_filter})
gyroscope_filter = pd.DataFrame({"t": imu_time, "gx": gx_filter, "gy": gy_filter, "gz": gz_filter})
magnetometer_filter = pd.DataFrame({"t": imu_time, "mx": mx_filter, "my": my_filter, "mz": mz_filter})

num_samples = len(ax)  # Počet vzorků

# Inicializace filtrů pro odhad quaternionů
madgwick = ahrs.filters.Madgwick()
fourati = ahrs.filters.Fourati()
ekf = ahrs.filters.EKF()
test = ahrs.filters.Complementary()

# Inicializace quaternionů pro jednotlivé filtry
Q_test = np.zeros((num_samples, 4))
Q_test[0] = [1.0, 0.0, 0.0, 0.0]

Q_ekf = np.zeros((num_samples, 4))
Q_ekf[0] = [1.0, 0.0, 0.0, 0.0]

Q_madwick = np.zeros((num_samples, 4))
Q_madwick[0] = [1.0, 0.0, 0.0, 0.0]

Q_fourati = np.zeros((num_samples, 4))
Q_fourati[0] = [1.0, 0.0, 0.0, 0.0]

# Seznam pro vykreslování eulerových úhlů estimovaných pomocí jednotlivých filtrů
display_y = ["madgwick", "fourati", "ekf"]

# Algoritmus pro výpočet quaternionů pro všechny filtry
for i in range(1, num_samples):
    madgwick.Dt = delta_t[i]
    fourati.Dt = delta_t[i]
    ekf.Dt = delta_t[i]
    test.Dt = delta_t[i]
    fourati.gain = 0.025
    madgwick.gain = 0.001
    Q_madwick[i] = madgwick.updateIMU(Q_madwick[i - 1], gyr=np.array([gx_filter[i], gy_filter[i], gz_filter[i]]),
                                      acc=np.array([ay_filter[i], ax_filter[i], -az_filter[i]]))

    Q_fourati[i] = fourati.update(Q_fourati[i - 1], gyr=np.array([gx_filter[i], gy_filter[i], gz_filter[i]]),
                                  acc=np.array([ay_filter[i], ax_filter[i], -az_filter[i]]),
                                  mag=np.array([mx_filter[i] * 0.1, my_filter[i] * 0.1, mz_filter[i] * 0.1]))

    Q_ekf[i] = ekf.update(Q_ekf[i - 1], gyr=np.array([gx_filter[i], gy_filter[i], gz_filter[i]]),
                          acc=np.array([ax_filter[i], ay_filter[i], az_filter[i]]),
                          mag=np.array([mx_filter[i] * 100, my_filter[i] * 100, mz_filter[i] * 100]))

    Q_test[i] = test.update(Q_test[i - 1], gyr=np.array([gx_filter[i], gy_filter[i], gz_filter[i]]),
                            acc=np.array([ay_filter[i], ax_filter[i], -az_filter[i]]),
                            mag=np.array([mx_filter[i] * 100, my_filter[i] * 100, mz_filter[i] * 100]))

# Vytvoření DataFrame obsahující eulerovy úhly pro každý filtr
euler_ekf = quaternion_get_dataframe(Q_ekf, num_samples)
euler_madgwick = quaternion_get_dataframe(Q_madwick, num_samples)
euler_fourati = quaternion_get_dataframe(Q_fourati, num_samples)
euler_test = quaternion_get_dataframe(Q_test, num_samples)

# DataFrame s roll úhlem pro každý filtr
roll_my_estimator = pd.DataFrame({"t": imu_time, "ekf": euler_ekf["roll"], "madgwick": euler_madgwick["roll"],
                                  "fourati": euler_fourati["roll"], "test": euler_test["roll"]})
# DataFrame s pitch úhlem pro každý filtr
pitch_my_estimator = pd.DataFrame({"t": imu_time, "ekf": euler_ekf["pitch"], "madgwick": euler_madgwick["pitch"],
                                   "fourati": euler_fourati["pitch"], "test": euler_test["pitch"]})
# DataFrame s yaw úhlem pro každý filtr
yaw_my_estimator = pd.DataFrame({"t": imu_time, "ekf": euler_ekf["yaw"], "madgwick": euler_madgwick["yaw"],
                                 "fourati": euler_fourati["yaw"], "test": euler_test["yaw"]})

# Offset od možné reference vypočítané pomocí EKF
add_value1 = roll_my_estimator.loc[roll_my_estimator["t"] > 1000000, "ekf"].iloc[0]
add_value2 = pitch_my_estimator.loc[pitch_my_estimator["t"] > 1000000, "ekf"].iloc[0]
add_value3 = yaw_my_estimator.loc[yaw_my_estimator["t"] > 1000000, "ekf"].iloc[0]

# Aplikace offsetu na estimované úhly
roll_my_estimator.loc[
    roll_my_estimator["t"] > 1000000, roll_my_estimator.columns.difference(['t', 'ekf'])] += add_value1
pitch_my_estimator.loc[
    pitch_my_estimator["t"] > 1000000, pitch_my_estimator.columns.difference(['t', 'ekf'])] += add_value2
yaw_my_estimator.loc[yaw_my_estimator["t"] > 1000000, yaw_my_estimator.columns.difference(['t', 'ekf'])] += add_value3

# Interpolace estimovaných úhlů vypočítaných pomocí UAV algoritmu na časovou osu IMU
roll_interp = np.interp(imu_time, board_time, roll_estimated)
pitch_interp = np.interp(imu_time, board_time, pitch_estimated)
yaw_interp = np.interp(imu_time, board_time, yaw_estimated)

# Vytvoření DataFrame s interpolovanými úhly vypočítanými pomocí UAV algoritmu
roll_board_estimator_interp = pd.DataFrame({"t": imu_time, "roll_odo_interp": roll_interp})
pitch_board_estimator_interp = pd.DataFrame({"t": imu_time, "pitch_odo_interp": pitch_interp})
yaw_board_estimator_interp = pd.DataFrame({"t": imu_time, "yaw_odo_interp": yaw_interp})

# Vytvoření DataFrame s estimovanými úhly vypočítanými pomocí UAV algoritmu
roll_board_estimator = pd.DataFrame({"t": board_time, "roll_odo": roll_estimated})
pitch_board_estimator = pd.DataFrame({"t": board_time, "pitch_odo": pitch_estimated})
yaw_board_estimator = pd.DataFrame({"t": board_time, "yaw_odo": yaw_estimated})

# Filtrace dat podle skip_time
roll_my_estimator = roll_my_estimator.loc[roll_my_estimator["t"] > skip_time]
pitch_my_estimator = pitch_my_estimator.loc[pitch_my_estimator["t"] > skip_time]
yaw_my_estimator = yaw_my_estimator.loc[yaw_my_estimator["t"] > skip_time]
roll_my_estimator.reset_index(drop=True, inplace=True)
pitch_my_estimator.reset_index(drop=True, inplace=True)
yaw_my_estimator.reset_index(drop=True, inplace=True)
roll_my_estimator["t"] -= roll_my_estimator.loc[0, "t"]
pitch_my_estimator["t"] -= pitch_my_estimator.loc[0, "t"]
yaw_my_estimator["t"] -= yaw_my_estimator.loc[0, "t"]

# Filtrace interpolovaných dat podle skip_time
roll_board_estimator_interp = roll_board_estimator_interp.loc[roll_board_estimator_interp["t"] > skip_time]
pitch_board_estimator_interp = pitch_board_estimator_interp.loc[pitch_board_estimator_interp["t"] > skip_time]
yaw_board_estimator_interp = yaw_board_estimator_interp.loc[yaw_board_estimator_interp["t"] > skip_time]
roll_board_estimator_interp.reset_index(drop=True, inplace=True)
pitch_board_estimator_interp.reset_index(drop=True, inplace=True)
yaw_board_estimator_interp.reset_index(drop=True, inplace=True)
roll_board_estimator_interp["t"] -= roll_board_estimator_interp.loc[0, "t"]
pitch_board_estimator_interp["t"] -= pitch_board_estimator_interp.loc[0, "t"]
yaw_board_estimator_interp["t"] -= yaw_board_estimator_interp.loc[0, "t"]

# Filtrace dat z akcelerometru podle skip_time
accelerometer = accelerometer.loc[accelerometer["t"] > skip_time]
accelerometer.reset_index(drop=True, inplace=True)
accelerometer["t"] -= accelerometer.loc[0, "t"]
accelerometer_filter = accelerometer_filter.loc[accelerometer_filter["t"] > skip_time]
accelerometer_filter.reset_index(drop=True, inplace=True)
accelerometer_filter["t"] -= accelerometer_filter.loc[0, "t"]
ax_filter = accelerometer_filter["ax"]
ay_filter = accelerometer_filter["ay"]
az_filter = accelerometer_filter["az"]

# Filtrace dat z gyroskopu podle skip_time
gyroscope = gyroscope.loc[gyroscope["t"] > skip_time]
gyroscope.reset_index(drop=True, inplace=True)
gyroscope["t"] -= gyroscope.loc[0, "t"]
gyroscope_filter = gyroscope_filter.loc[gyroscope_filter["t"] > skip_time]
gyroscope_filter.reset_index(drop=True, inplace=True)
gyroscope_filter["t"] -= gyroscope_filter.loc[0, "t"]

# Filtrace dat z magnetometru podle skip_time
magnetometer = magnetometer.loc[magnetometer["t"] > skip_time]
magnetometer.reset_index(drop=True, inplace=True)
magnetometer["t"] -= magnetometer.loc[0, "t"]
magnetometer_filter = magnetometer_filter.loc[magnetometer_filter["t"] > skip_time]
magnetometer_filter.reset_index(drop=True, inplace=True)
magnetometer_filter["t"] -= magnetometer_filter.loc[0, "t"]

# Filtrace GPS dat podle skip_time
gps = gps.loc[gps["timestamp"] > skip_time]
gps.reset_index(drop=True, inplace=True)
gps["timestamp"] = gps["timestamp"] - gps.loc[0, "timestamp"]

# Výběr odhadovaných úhlů pomocí všech filtrů a
# podle úhlů estimovaných pomocí UAV algoritmu,
# které je možné vybrat, ale nejsou platné
# kvůli nesplnění podmínek estimace polohy pouze z IMU, a proto je nepoužívame

if ahrs_alg == "ekf":
    # Eulerovy úhly estimované pomocí rozšířeného kalmanova filtru
    roll = roll_my_estimator["ekf"]
    pitch = pitch_my_estimator["ekf"]
    yaw = yaw_my_estimator["ekf"]

elif ahrs_alg == "fourati":
    # Eulerovy úhly estimované pomocí fouratiho filtru
    roll = roll_my_estimator["fourati"]
    pitch = pitch_my_estimator["fourati"]
    yaw = yaw_my_estimator["fourati"]

elif ahrs_alg == "test":
    # Eulerovy úhly estimované pomocí komplementárního filtru
    roll = roll_my_estimator["test"]
    pitch = pitch_my_estimator["test"]
    yaw = yaw_my_estimator["test"]

elif ahrs_alg == "madgwick":
    # Eulerovy úhly estimované pomocí madgwick filtru
    roll = roll_my_estimator["madgwick"]
    pitch = pitch_my_estimator["madgwick"]
    yaw = yaw_my_estimator["madgwick"]

elif ahrs_alg == "uav":
    # Eulerovy úhly estimované z UAV algoritmu
    roll = roll_board_estimator_interp["roll_odo_interp"]
    pitch = pitch_board_estimator_interp["pitch_odo_interp"]
    yaw = yaw_board_estimator_interp["yaw_odo_interp"]

else:
    raise ValueError(f"Unknown AHRS algorithm: {ahrs_alg}")

# Transformace akcelerometrických dat do inerciálního souřadného systému
Ax_I = 0 * ax_filter
Ay_I = 0 * ax_filter
Az_I = 0 * ax_filter
for i in range(0, len(roll)):
    ax_ = ax_filter[i]
    ay_ = ay_filter[i]
    az_ = az_filter[i]
    ab = np.array([ax_, ay_, az_])
    phi = roll[i] * np.pi / 180
    theta = pitch[i] * np.pi / 180
    psi = yaw[i] * np.pi / 180
    ctheta = np.cos(theta)
    cpsi = np.cos(psi)
    sphi = np.sin(phi)
    stheta = np.sin(theta)
    cphi = np.cos(phi)
    spsi = np.sin(psi)
    ttheta = np.tan(theta)
    matrix_rot = np.asarray([[ctheta * cpsi, sphi * stheta * cpsi - cphi * spsi, cphi * stheta * cpsi + sphi * spsi],
                             [ctheta * spsi, sphi * stheta * spsi + cphi * cpsi, cphi * stheta * spsi - sphi * cpsi],
                             [-stheta, sphi * ctheta, cphi * ctheta]])
    NED_conv = np.asarray([[0, 1, 0],
                           [1, 0, 0],
                           [0, 0, -1]])
    matrix_rot = np.dot(NED_conv, matrix_rot)

    aI = np.dot(matrix_rot, ab)
    Ax_I[i] = aI[0]
    Ay_I[i] = aI[1]
    Az_I[i] = (aI[2] - 9.81)
local_acceleration = pd.DataFrame({"t": imu_time, "ax": Ax_I, "ay": Ay_I, "az": Az_I})

# Výpočet rychlosti z akcelerometrických dat
vx = 0 * Ax_I
vy = 0 * Ay_I
vz = 0 * Az_I
for i in range(0, len(Ax_I) - 1):
    vx[i + 1] = vx[i] + Ax_I[i] * delta_t[i]
    vy[i + 1] = vy[i] + Ay_I[i] * delta_t[i]
    vz[i + 1] = vz[i] + Az_I[i] * delta_t[i]
local_velocity = pd.DataFrame({"t": imu_time, "vx": vx, "vy": vy, "vz": vz})

# Výpočet polohy z rychlostních dat
px = 0 * Ax_I
py = 0 * Ay_I
pz = 0 * Az_I
for i in range(0, len(Ax_I) - 1):
    px[i + 1] = px[i] + vx[i] * delta_t[i]
    py[i + 1] = py[i] + vy[i] * delta_t[i]
    pz[i + 1] = pz[i] + vz[i] * delta_t[i]
local_position = pd.DataFrame({"t": imu_time, "px": px, "py": py, "pz": pz})

# Normalizace časové osy eulerových úhlů na sekundy
roll_my_estimator["t"] /= 1000000
pitch_my_estimator["t"] /= 1000000
yaw_my_estimator["t"] /= 1000000
roll_board_estimator_interp["t"] /= 1000000
pitch_board_estimator_interp["t"] /= 1000000
yaw_board_estimator_interp["t"] /= 1000000
roll_board_estimator["t"] /= 1000000
pitch_board_estimator["t"] /= 1000000
yaw_board_estimator["t"] /= 1000000

# Vytvoření DataFrame s polohou IMU
imu_position = pd.DataFrame({"t": imu_time, "x": px, "y": py})
gps_converted = pd.DataFrame()

# Konverze GPS souřadnic na UTM a jejich normalizace
utm_data = gps.apply(lambda row: convert_gps_to_utm(row["lat"] / 10000000, row["lon"] / 10000000), axis=1)
gps_converted[["x", "y"]] = pd.DataFrame(utm_data.tolist(), index=gps.index)
gps_converted["x"] -= gps_converted.loc[0, "x"]
gps_converted["y"] -= gps_converted.loc[0, "y"]
gps_converted["t"] = gps["timestamp"]

# Uložení estimovaných dat pomocí IMU a souřadnic z GPS do souboru
imu_position.to_csv(ros2_path_imu, index=False)
gps_converted.to_csv(ros2_path_gps, index=False)

# Nalezení nejbližších časových bodů a následné filtrování dle tohoto kritéria
filtered_df = pd.DataFrame(columns=["t", "x1", "y1", "x2", "y2"])
for _, row in gps_converted.iterrows():
    nearest_row = imu_position.iloc[(imu_position["t"] - row["t"]).abs().argsort()[:1]]

    filtered_df = pd.concat([filtered_df, pd.DataFrame({"t": [row["t"]],
                                                        "x1": [row["x"]],
                                                        "y1": [row["y"]],
                                                        "x2": nearest_row["x"].values[0],
                                                        "y2": nearest_row["y"].values[0]})], ignore_index=True)

filtered_df = pd.DataFrame(
    {"t": filtered_df["t"], "x1": filtered_df["x1"], "y1": filtered_df["y1"], "x2": filtered_df["x2"],
     "y2": filtered_df["y2"]})

# Výpočet euklidovské vzdálenosti mezi polohou určenou pomocí IMU a GPS pozicí
dif_x = filtered_df["x2"] - filtered_df["x1"]
dif_y = filtered_df["y2"] - filtered_df["y1"]

euclid_distance = np.sqrt(dif_x ** 2 + dif_y ** 2)
print(f"Mean euclidian distance: {np.mean(euclid_distance):.2f}")

# Nastavení velikosti grafu
figure_size = (10, 5)

if "gyro" in plot_data:
    # Vykreslení nefiltrovaných dat z gyroskopu
    gyroscope.plot(x="t", y=["gx", "gy", "gz"], grid=True, figsize=figure_size, ylabel="w [rad/s]", xlabel="t [us]",
                   title="Angular Rate")

if "gyro_filter" in plot_data:
    # Vykreslení filtrovaných dat z gyroskopu
    gyroscope_filter.plot(x="t", y=["gx", "gy", "gz"], grid=True, figsize=figure_size, ylabel="w [rad/s]",
                          xlabel="t [us]",
                          title="Filtered Angular Rate")

if "acc" in plot_data:
    # Vykreslení nefiltrovaných dat z akcelerometru
    accelerometer.plot(x="t", y=["ax", "ay", "az"], grid=True, figsize=figure_size, ylabel="a [m/s2]", xlabel="t [us]",
                       title="Acceleration")

if "acc_filter" in plot_data:
    # Vykreslení filtrovaných dat z akcelerometru
    accelerometer_filter.plot(x="t", y=["ax", "ay", "az"], grid=True, figsize=figure_size, ylabel="a [m/s2]",
                              xlabel="t [us]",
                              title="Filtered Acceleration")

if "mag" in plot_data:
    # Vykreslení nefiltrovaných dat z magnetometru
    magnetometer.plot(x="t", y=["mx", "my", "mz"], grid=True, figsize=figure_size, ylabel="B [Gs]", xlabel="t [us]",
                      title="Magnetic Induction")

if "mag_filter" in plot_data:
    # Vykreslení filtrovaných dat z magnetometru
    magnetometer_filter.plot(x="t", y=["mx", "my", "mz"], grid=True, figsize=figure_size, ylabel="B [Gs]",
                             xlabel="t [us]",
                             title="Filtered Magnetic Induction")

if "local_acc" in plot_data:
    # Vykreslení inerciálních zrychlení
    local_acceleration.plot(x="t", y=["ax", "ay", "az"], grid=True, figsize=figure_size, ylabel="a [m/s2]",
                            xlabel="t [us]",
                            title="Local Acceleration")

if "local_velocity" in plot_data:
    # Vykreslení inerciálních rychlostí
    local_velocity.plot(x="t", y=["vx", "vy", "vz"], grid=True, figsize=figure_size, ylabel="v [m/s]", xlabel="t [us]",
                        title="Local Velocity")

if "local_position" in plot_data:
    # Vykreslení inerciálních poloh
    local_position.plot(x="t", y=["px", "py", "pz"], grid=True, figsize=figure_size, ylabel="p [m]", xlabel="t [us]",
                        title="Local Position")

if "trajectory" in plot_data:
    # Vykreslení trajektorie UAV určených pomocí GPS a algoritmu pro estimaci polohy pomocí IMU
    _, ax = plt.subplots()
    gps_converted.plot(x="x", y="y", ax=ax)
    imu_position.plot(x="x", y="y", ax=ax, grid=True, legend=False, ylabel="y [m]", xlabel="x [m]")
    plt.plot(imu_position.loc[0, "x"], imu_position.loc[0, "y"], marker='o', color='r')
    plt.legend(["GPS", "IMU estimace"], loc="lower right")

if "trajectory_precision" in plot_data:
    # Vykreslení odchylky od referenční trajektorie vypočítané pomocí euklidovské vzdálenosti v závislosti na čase
    plt.figure()
    plt.plot(filtered_df["t"] / 1000000, euclid_distance)
    plt.xlabel("t [s]")
    plt.ylabel("s [m]")
    plt.grid(True)
    plt.title("Odchylka od referenční trajektorie závislá na čase")

if "euler_angles" in plot_data:
    # Vykreslení Eulerových úhlů
    fig_roll, ax_roll = plt.subplots()
    roll_my_estimator.plot(x="t", y=[ahrs_alg], ax=ax_roll)
    roll_board_estimator_interp.plot(x="t", y="roll_odo_interp", ax=ax_roll, grid=True, ylabel="úhel [°]",
                                     xlabel="t [s]",
                                     title="Roll")
    ax_roll.legend([f"{ahrs_alg.capitalize()} estimátor", "Reference ze Simulátoru"], loc='lower right')

    fig_pitch, ax_pitch = plt.subplots()
    pitch_my_estimator.plot(x="t", y=[ahrs_alg], ax=ax_pitch)
    pitch_board_estimator_interp.plot(x="t", y="pitch_odo_interp", ax=ax_pitch, grid=True, ylabel="úhel [°]",
                                      xlabel="t [s]",
                                      title="Pitch")
    ax_pitch.legend([f"{ahrs_alg.capitalize()} estimátor", "Reference ze Simulátoru"], loc='lower right')

    fig_yaw, ax_yaw = plt.subplots()
    yaw_my_estimator.plot(x="t", y=[ahrs_alg], ax=ax_yaw)
    yaw_board_estimator_interp.plot(x="t", y="yaw_odo_interp", ax=ax_yaw, grid=True, ylabel="úhel [°]", xlabel="t [s]",
                                    title="Yaw")
    ax_yaw.legend([f"{ahrs_alg.capitalize()} estimátor", "Reference ze Simulátoru"], loc='lower right')

plt.show()
