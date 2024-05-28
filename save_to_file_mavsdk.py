import asyncio

from mavsdk import System


def open_and_write_header(path, header):
    file = open(path, "w")
    file.write(header + '\n')
    file.close()

    file = open(path, "a")
    return file


file_gps = open_and_write_header("imu/gps.txt", "timestamp,lat,lon,alt")
file_odo = open_and_write_header("imu/odo.txt", "timestamp,yaw_odo,roll_odo,pitch_odo")
file_imu = open_and_write_header("imu/sensor_combined.txt", "timestamp,ax,ay,az,gx,gy,gz,mx,my,mz")


async def run():
    # Init the drone
    drone = System()
    await drone.connect(system_address="udp://:14540")

    asyncio.ensure_future(print_imu(drone))
    asyncio.ensure_future(print_gps(drone))
    asyncio.ensure_future(print_odometry(drone))

    while True:
        await asyncio.sleep(0.1)


async def print_imu(drone):
    async for imu in drone.telemetry.imu():
        file_imu.write(
            f"{imu.timestamp_us},"
            f"{imu.acceleration_frd.forward_m_s2},{imu.acceleration_frd.right_m_s2},{imu.acceleration_frd.down_m_s2},"
            f"{imu.angular_velocity_frd.forward_rad_s},{imu.angular_velocity_frd.right_rad_s},{imu.angular_velocity_frd.down_rad_s},"
            f"{imu.magnetic_field_frd.forward_gauss},{imu.magnetic_field_frd.right_gauss},{imu.magnetic_field_frd.down_gauss}\n")


async def print_gps(drone):
    async for gps in drone.telemetry.raw_gps():
        file_gps.write(
            f"{gps.timestamp_us},{gps.latitude_deg},{gps.longitude_deg},{gps.absolute_altitude_m}\n")


async def print_odometry(drone):
    async for odometry in drone.telemetry.attitude_euler():
        file_odo.write(
            f"{odometry.timestamp_us},{odometry.yaw_deg},{odometry.roll_deg},{odometry.pitch_deg}\n")


if __name__ == "__main__":
    asyncio.run(run())
