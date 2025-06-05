from dronekit import connect, VehicleMode, LocationGlobalRelative
import time
import math

# ---------------------------------------------
# Function to get user (target) position
# ---------------------------------------------
def get_user_position():
    """
    Returns static lat, lon, alt for testing.
    """
    # Target location ~300 meters away from the starting point for a noticeable flight
    lat = -35.360500  # About 300m north of start
    lon = 149.168000  # About 250m east of start  
    alt = 15  # meters above home (higher altitude too)
    return lat, lon, alt

# ---------------------------------------------
# Function to calculate distance to target (Haversine)
# ---------------------------------------------
def get_distance_meters(lat1, lon1, lat2, lon2):
    R = 6371000  # Earth radius in meters
    phi1 = math.radians(lat1)
    phi2 = math.radians(lat2)
    delta_phi = math.radians(lat2 - lat1)
    delta_lambda = math.radians(lon2 - lon1)
    a = math.sin(delta_phi/2)**2 + math.cos(phi1)*math.cos(phi2)*math.sin(delta_lambda/2)**2
    c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))
    return R * c

# ---------------------------------------------
# Function to pre-check and fly to target
# ---------------------------------------------
def go_to_target(lat, lon, alt, vehicle):
    """
    Performs pre-flight checks and flies to target GPS position if safe.
    """
    print("[INFO] Running pre-flight checks...")

    if not vehicle.is_armable:
        print("[ERROR] Vehicle is not armable.")
        return

    battery = vehicle.battery.level
    if battery is not None and battery < 30:
        print(f"[ERROR] Battery too low: {battery}%")
        return

    current_location = vehicle.location.global_relative_frame
    print(f"[INFO] Current drone location: ({current_location.lat}, {current_location.lon}, {current_location.alt}m)")
    print(f"[INFO] Target location: ({lat}, {lon}, {alt}m)")
    distance = get_distance_meters(current_location.lat, current_location.lon, lat, lon)

    if distance > 2000:
        print(f"[ERROR] Target too far: {distance:.1f}m")
        return

    print("[INFO] Pre-flight check passed.")
    print(f"[INFO] Flying {distance:.1f}m to ({lat}, {lon}, alt {alt}m)...")

    # Takeoff if not already flying
    if vehicle.location.global_relative_frame.alt < 2:
        vehicle.mode = VehicleMode("GUIDED")
        vehicle.armed = True
        while not vehicle.armed:
            print("[INFO] Waiting for arming...")
            time.sleep(1)
        vehicle.simple_takeoff(alt)
        while vehicle.location.global_relative_frame.alt < alt * 0.95:
            print(f"[INFO] Ascending: {vehicle.location.global_relative_frame.alt:.1f}m")
            time.sleep(1)

    # Fly to target
    target = LocationGlobalRelative(lat, lon, alt)
    vehicle.simple_goto(target)

# ---------------------------------------------
# Main function
# ---------------------------------------------
def main():
    print("[INFO] Connecting to vehicle...")
    vehicle = connect('127.0.0.1:14550', wait_ready=True)

    lat, lon, alt = get_user_position()
    go_to_target(lat, lon, alt, vehicle)

    print("[INFO] Mission complete. Holding position.")
    # Leave vehicle connected, or add vehicle.close() if done

if __name__ == "__main__":
    main()
