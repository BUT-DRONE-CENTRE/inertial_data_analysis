import ahrs
import numpy as np
import pandas as pd
import pyproj
from matplotlib import pyplot as plt


def get_rpy(Q, samples):
    """
    Převádí quaterniony na úhly roll, pitch a yaw.

    Parametry:
    ----------
    Q : ndarray
        Matice quaternionů (qw, qx, qy, qz).

    samples : int
        Počet vzorků

    Návratová hodnota:
    -------------------
    list
        Seznam obsahující tři pole: roll, pitch a yaw ve stupních.
    """
    roll = np.zeros(samples)
    pitch = np.zeros(samples)
    yaw = np.zeros(samples)
    for i in range(samples):
        [r, p, y] = ahrs.common.orientation.q2rpy(Q[i], in_deg=True)
        roll[i] = r
        pitch[i] = p
        yaw[i] = y
    return [roll, pitch, yaw]


def quaternion_get_dataframe(Q, samples):
    """
    Z Quaternionů vytvoří DataFrame, jehož obsah budou úhly roll, pitch a yaw.

    Parametry:
    ----------
    Q : ndarray
        Matice quaternionů.

    Návratová hodnota:
    -------------------
    DataFrame
        DataFrame obsahující roll, pitch a yaw ve stupních.
    """
    [r, p, y] = get_rpy(Q, samples)
    data_frame = pd.DataFrame({"roll": r, "pitch": p, "yaw": y})
    return data_frame


def exp_filter(vec, alfa):
    """
    Filtruje vektor pomocí jednoduchého exponenciálního vyrovnávácího filtru.

    Parametry:
    ----------
    vec : ndarray
        Vstupní vektor k filtrování.
    alfa : float
        Filtrační konstanta (0 < alfa < 1).

    Návratová hodnota:
    -------------------
    ndarray
        Filtrovaný vektor.
    """
    outvec = 0 * vec
    outvec[0] = vec[0]
    for x in range(1, len(vec)):
        outvec[x] = (1 - alfa) * outvec[x - 1] + alfa * vec[x]
    return outvec


def convert_gps_to_utm(lat, lon):
    """
    Převádí GPS souřadnice na UTM souřadnice.

    Parametry:
    ----------
    lat : float
        Zeměpisná šířka.
    lon : float
        Zeměpisná délka.

    Návratová hodnota:
    -------------------
    tuple
        Dvojice UTM souřadnic (x, y).
    """
    wgs84 = pyproj.CRS("EPSG:4326")
    utm_zone = int((lon + 180) / 6) + 1
    utm_crs = pyproj.CRS.from_proj4(f"+proj=utm +zone={utm_zone} +ellps=WGS84 +datum=WGS84 +units=m +no_defs")
    transformer = pyproj.Transformer.from_crs(wgs84, utm_crs, always_xy=True)
    utm_x, utm_y = transformer.transform(lon, lat)
    return utm_x, utm_y
