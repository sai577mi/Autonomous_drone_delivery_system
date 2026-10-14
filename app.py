from flask import Flask, request, jsonify, render_template_string
import math
import random
import heapq
import sqlite3
import threading
import time
import uuid
import json
import os
import secrets
import base64
from io import BytesIO
from datetime import datetime, timezone
from urllib.request import urlopen
from urllib.parse import urlencode, quote_plus

try:
    import qrcode
except ImportError:
    qrcode = None

app = Flask(__name__)

# ============================================================
# SKYDELIVER PRO
# Autonomous Multi-Drone Last-Mile Delivery Application
#
# Mission:
# BASE -> PICKUP -> CUSTOMER -> RETURN -> CHARGE
#
# Features:
# - Automatic live weather using Open-Meteo
# - 6-drone fleet
# - 240+ generated customer delivery points
# - A* route planning
# - Wind vector + heading compensation
# - Simulated roll/pitch/yaw attitude
# - Battery + charging
# - Multiple simultaneous missions
# - SQLite delivery history
# - Fleet statistics
# - Live Leaflet map
# ============================================================

DB_FILE = "skydeliver_history.db"
WEATHER_REFRESH_SECONDS = 60
SIMULATION_TICK_SECONDS = 1.0

# Hyderabad operating area
CITY_LAT = 17.4300
CITY_LON = 78.4300

BASES = {
    "Central Drone Base": {"name": "Central Drone Base", "lat": 17.3850, "lon": 78.4867},
    "Hitech Drone Base": {"name": "Hitech Drone Base", "lat": 17.4480, "lon": 78.3940},
    "Gachibowli Drone Base": {"name": "Gachibowli Drone Base", "lat": 17.4400, "lon": 78.3500},
}


# ============================================================
# HYPER-LOCAL WEATHER ZONES (ACADEMIC SIMULATION)
# Different Hyderabad areas intentionally have different weather.
# The real Open-Meteo reading is used as the baseline, then the drone
# receives a coordinate-based local weather overlay while travelling.
# ============================================================
DEMO_CLEAR_WEATHER = True

WEATHER_ZONES = [
    {"name":"Jubilee Hills Rain Cell","lat":17.4326,"lon":78.4071,"radius_km":2.2,"condition":"HEAVY RAIN","rain":9.5,"wind":22,"gust":34,"cloud":96},
    {"name":"Hitech City Storm Cell","lat":17.4480,"lon":78.3940,"radius_km":2.0,"condition":"THUNDERSTORM","rain":4.5,"wind":52,"gust":68,"cloud":98},
    {"name":"Gachibowli Clear Zone","lat":17.4400,"lon":78.3500,"radius_km":2.4,"condition":"CLEAR","rain":0.0,"wind":9,"gust":15,"cloud":18},
    {"name":"Banjara Hills Wind Zone","lat":17.4156,"lon":78.4347,"radius_km":1.8,"condition":"STRONG WIND","rain":0.2,"wind":38,"gust":50,"cloud":48},
    {"name":"Charminar Storm Cell","lat":17.3616,"lon":78.4747,"radius_km":1.8,"condition":"STORM","rain":6.5,"wind":48,"gust":63,"cloud":92},
    {"name":"Secunderabad Rain Cell","lat":17.4399,"lon":78.4983,"radius_km":2.0,"condition":"RAIN","rain":5.0,"wind":18,"gust":28,"cloud":90},
]

def local_weather_for_position(position, base_weather, step=0):
    """Return mission weather.
    Demo mode keeps the flight corridor sunny/clear while the dashboard
    continues to display the automatic Open-Meteo reading."""
    w = dict(base_weather or {})
    if DEMO_CLEAR_WEATHER:
        w.update({
            "condition": "Sunny / Clear",
            "emoji": "☀️",
            "rain": 0.0,
            "precipitation": 0.0,
            "wind_speed": 9.0,
            "wind_gusts": 15.0,
            "cloud_cover": 18,
            "weather_code": 0,
            "zone": "CLEAR SKY DEMO CORRIDOR",
            "zone_distance_km": None,
            "simulated_local": True,
            "weather_alert": "🟢 Sunny / clear weather • normal flight corridor",
        })
        return w
    best = None
    best_d = 1e9
    for z in WEATHER_ZONES:
        d = haversine(position["lat"], position["lon"], z["lat"], z["lon"])
        if d <= z["radius_km"] and d < best_d:
            best, best_d = z, d
    if best:
        wave = math.sin(step / 4.0)
        gust_wave = math.sin(step / 3.0 + 0.8)
        w["wind_speed"] = round(max(0, best["wind"] + wave * 4), 1)
        w["wind_gusts"] = round(max(w["wind_speed"], best["gust"] + gust_wave * 6), 1)
        w["rain"] = round(max(0, best["rain"] + wave * 1.2), 1)
        w["cloud_cover"] = best["cloud"]
        w["condition"] = best["condition"]
        w["zone"] = best["name"]
        w["zone_distance_km"] = round(best_d, 2)
        w["simulated_local"] = True
        w["weather_alert"] = (
            f"⚠️ {best['condition']} in {best['name']} • {w['wind_speed']} km/h wind • "
            f"gust {w['wind_gusts']} km/h • rain {w['rain']} mm"
        )
    else:
        # Normal area: small variation around the live baseline.
        w["wind_speed"] = round(max(0, float(w.get("wind_speed") or 0) + math.sin(step/5.0)*3), 1)
        w["wind_gusts"] = round(max(w["wind_speed"], float(w.get("wind_gusts") or 0) + math.sin(step/4.0)*4), 1)
        w["rain"] = round(max(0, float(w.get("rain") or 0) + math.sin(step/6.0)*0.4), 1)
        w["zone"] = "NORMAL HYDERABAD WEATHER"
        w["zone_distance_km"] = None
        w["simulated_local"] = True
        w["weather_alert"] = "🟢 Normal local weather corridor"
    return w

DRONES = {
    "FOOD-01": {"id": "FOOD-01", "type": "food", "name": "Food Drone 01", "icon": "🍔", "max_speed": 48, "battery": 96.0, "base": "Central Drone Base", "busy": False, "mission_id": None, "health": 95, "gps_signal": 97, "connection": 95, "temperature": 32, "flight_hours": 40.0, "altitude": 65, "range_km": 20, "max_payload_kg": 2.0},
    "FOOD-02": {"id": "FOOD-02", "type": "food", "name": "Food Drone 02", "icon": "🍔", "max_speed": 46, "battery": 91.0, "base": "Hitech Drone Base", "busy": False, "mission_id": None, "health": 95, "gps_signal": 97, "connection": 95, "temperature": 32, "flight_hours": 40.0, "altitude": 65, "range_km": 20, "max_payload_kg": 2.0},
    "MED-01": {"id": "MED-01", "type": "medical", "name": "Medical Drone 01", "icon": "🏥", "max_speed": 55, "battery": 94.0, "base": "Central Drone Base", "busy": False, "mission_id": None, "health": 95, "gps_signal": 97, "connection": 95, "temperature": 32, "flight_hours": 40.0, "altitude": 65, "range_km": 20, "max_payload_kg": 3.0},
    "MED-02": {"id": "MED-02", "type": "medical", "name": "Medical Drone 02", "icon": "🏥", "max_speed": 53, "battery": 89.0, "base": "Gachibowli Drone Base", "busy": False, "mission_id": None, "health": 95, "gps_signal": 97, "connection": 95, "temperature": 32, "flight_hours": 40.0, "altitude": 65, "range_km": 20, "max_payload_kg": 3.0},
    "GAD-01": {"id": "GAD-01", "type": "gadget", "name": "Gadget Drone 01", "icon": "📱", "max_speed": 60, "battery": 88.0, "base": "Hitech Drone Base", "busy": False, "mission_id": None, "health": 95, "gps_signal": 97, "connection": 95, "temperature": 32, "flight_hours": 40.0, "altitude": 65, "range_km": 20, "max_payload_kg": 4.0},
    "GAD-02": {"id": "GAD-02", "type": "gadget", "name": "Gadget Drone 02", "icon": "📱", "max_speed": 58, "battery": 84.0, "base": "Central Drone Base", "busy": False, "mission_id": None, "health": 95, "gps_signal": 97, "connection": 95, "temperature": 32, "flight_hours": 40.0, "altitude": 65, "range_km": 20, "max_payload_kg": 4.0},
}

PICKUPS = {
    "food": [
        {"id": "REST-01", "name": "Paradise Biryani Restaurant", "item": "Chicken Biryani", "lat": 17.4435, "lon": 78.4626},
        {"id": "REST-02", "name": "Hitech Food Court", "item": "Veg Biryani", "lat": 17.4483, "lon": 78.3915},
        {"id": "REST-03", "name": "Jubilee Food Hub", "item": "Pizza", "lat": 17.4326, "lon": 78.4071},
        {"id": "REST-04", "name": "Banjara Restaurant", "item": "Meals", "lat": 17.4156, "lon": 78.4347},
    ],
    "medical": [
        {"id": "MED-STORE-01", "name": "Apollo Pharmacy", "item": "Emergency Medicines", "lat": 17.4401, "lon": 78.3489},
        {"id": "MED-STORE-02", "name": "City Hospital", "item": "Medical Package", "lat": 17.4399, "lon": 78.4983},
        {"id": "MED-STORE-03", "name": "Health Care Pharmacy", "item": "Prescription Medicines", "lat": 17.3616, "lon": 78.4747},
    ],
    "gadget": [
        {"id": "GAD-STORE-01", "name": "Electronics Mega Store", "item": "Smartphone", "lat": 17.4483, "lon": 78.3915},
        {"id": "GAD-STORE-02", "name": "Tech World", "item": "Laptop", "lat": 17.4326, "lon": 78.4071},
        {"id": "GAD-STORE-03", "name": "Digital Hub", "item": "Headphones", "lat": 17.4156, "lon": 78.4347},
    ],
}

# Zones are deliberately populated with 15-35 individual delivery points.
# These are simulated application points, not claimed real addresses.
ZONES = {
    "Jubilee Hills": (17.4326, 78.4071, 35, 0.0070),
    "Banjara Hills": (17.4156, 78.4347, 30, 0.0070),
    "Hitech City": (17.4483, 78.3915, 30, 0.0070),
    "Madhapur": (17.4483, 78.3915, 25, 0.0060),
    "Gachibowli": (17.4401, 78.3489, 30, 0.0070),
    "Kukatpally": (17.4849, 78.4138, 25, 0.0070),
    "Begumpet": (17.4435, 78.4626, 20, 0.0060),
    "Secunderabad": (17.4399, 78.4983, 25, 0.0070),
    "Charminar": (17.3616, 78.4747, 20, 0.0060),
}

PARCEL_OFFICES = [
    {"name": "Central Parcel Office", "lat": 17.3910, "lon": 78.4930},
    {"name": "Hitech City Parcel Office", "lat": 17.4500, "lon": 78.3970},
    {"name": "Gachibowli Parcel Office", "lat": 17.4410, "lon": 78.3520},
]

CHARGERS = [
    {"name": "Central Charging Hub", "lat": 17.3890, "lon": 78.4870},
    {"name": "Hitech Charging Hub", "lat": 17.4480, "lon": 78.3940},
    {"name": "Gachibowli Charging Hub", "lat": 17.4400, "lon": 78.3500},
]

NO_FLY_ZONES = [
    {"id":"NFZ-01","name":"Restricted Government Zone","lat":17.4115,"lon":78.4550,"radius_km":0.85},
    {"id":"NFZ-02","name":"Airport Safety Buffer","lat":17.2400,"lon":78.4300,"radius_km":1.20},
    {"id":"NFZ-03","name":"Temporary Event Zone","lat":17.4390,"lon":78.4070,"radius_km":0.45},
]
system_alerts=[]

missions = {}
weather_state = {
    "ok": False,
    "condition": "Weather unavailable",
    "emoji": "🌐",
    "temperature": None,
    "rain": 0.0,
    "precipitation": 0.0,
    "wind_speed": 0.0,
    "wind_direction": 0.0,
    "wind_gusts": 0.0,
    "weather_code": None,
    "cloud_cover": 0,
    "updated_at": None,
    "source": "Open-Meteo",
    "error": None,
}

lock = threading.RLock()


# ============================================================
# DATABASE
# ============================================================

def db_connect():
    conn = sqlite3.connect(DB_FILE)
    conn.row_factory = sqlite3.Row
    return conn


def init_db():
    conn = db_connect()
    conn.execute("""
        CREATE TABLE IF NOT EXISTS deliveries (
            id TEXT PRIMARY KEY,
            drone_id TEXT NOT NULL,
            parcel_type TEXT NOT NULL,
            item TEXT,
            pickup_name TEXT,
            pickup_lat REAL,
            pickup_lon REAL,
            customer_id TEXT,
            customer_name TEXT,
            customer_lat REAL,
            customer_lon REAL,
            weather_condition TEXT,
            wind_speed REAL,
            wind_direction REAL,
            max_gust REAL,
            total_distance REAL,
            delivery_time_seconds REAL,
            status TEXT,
            created_at TEXT,
            completed_at TEXT,
            avg_speed REAL DEFAULT 0,
            weather_changes INTEGER DEFAULT 0,
            obstacle_events INTEGER DEFAULT 0
        )
    """)
    # Safe migrations for existing databases.
    existing = {row[1] for row in conn.execute("PRAGMA table_info(deliveries)").fetchall()}
    for name, typ in [("avg_speed", "REAL DEFAULT 0"), ("weather_changes", "INTEGER DEFAULT 0"), ("obstacle_events", "INTEGER DEFAULT 0")]:
        if name not in existing:
            conn.execute(f"ALTER TABLE deliveries ADD COLUMN {name} {typ}")
    conn.commit()
    conn.close()


init_db()


# ============================================================
# GEOGRAPHY
# ============================================================

def haversine(lat1, lon1, lat2, lon2):
    R = 6371.0
    p1 = math.radians(lat1)
    p2 = math.radians(lat2)
    dp = math.radians(lat2 - lat1)
    dl = math.radians(lon2 - lon1)
    a = math.sin(dp / 2) ** 2 + math.cos(p1) * math.cos(p2) * math.sin(dl / 2) ** 2
    return R * 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))


def dist(a, b):
    return haversine(a["lat"], a["lon"], b["lat"], b["lon"])


def bearing(a, b):
    lat1 = math.radians(a["lat"])
    lat2 = math.radians(b["lat"])
    dl = math.radians(b["lon"] - a["lon"])
    y = math.sin(dl) * math.cos(lat2)
    x = math.cos(lat1) * math.sin(lat2) - math.sin(lat1) * math.cos(lat2) * math.cos(dl)
    return (math.degrees(math.atan2(y, x)) + 360) % 360


def angle_diff(a, b):
    return (a - b + 540) % 360 - 180


def clamp(v, lo, hi):
    return max(lo, min(hi, v))


# ============================================================
# DELIVERY POINT GENERATOR
# ============================================================

def generate_delivery_points():
    points = []
    idx = 1
    # Deterministic pseudo-grid: easy to reproduce and visually spread.
    pattern = [
        (-1.00, -0.20), (-0.80, 0.35), (-0.65, -0.65), (-0.50, 0.10),
        (-0.35, 0.70), (-0.20, -0.85), (-0.10, 0.40), (0.00, -0.30),
        (0.10, 0.80), (0.20, -0.70), (0.30, 0.15), (0.40, 0.60),
        (0.50, -0.45), (0.62, 0.20), (0.74, -0.75), (0.84, 0.45),
        (0.94, -0.10), (-0.90, 0.78), (-0.72, -0.10), (-0.56, 0.50),
        (-0.42, -0.78), (-0.28, 0.18), (-0.14, 0.88), (0.02, 0.00),
        (0.16, -0.55), (0.32, 0.84), (0.46, -0.05), (0.58, 0.72),
        (0.70, -0.35), (0.82, 0.05), (0.96, 0.82), (-0.30, -0.25),
        (0.35, 0.35), (-0.65, 0.75), (0.68, 0.52),
    ]
    for zone, (lat, lon, count, spread) in ZONES.items():
        for i in range(count):
            x, y = pattern[i % len(pattern)]
            # Longitude degrees need a latitude correction.
            p_lat = lat + y * spread
            p_lon = lon + x * spread / max(math.cos(math.radians(lat)), 0.1)
            points.append({
                "id": f"{zone[:2].upper()}-{i+1:02d}",
                "zone": zone,
                "name": f"{zone} Delivery Point {i+1:02d}",
                "lat": round(p_lat, 6),
                "lon": round(p_lon, 6),
            })
            idx += 1
    return points


CUSTOMERS = generate_delivery_points()


# ============================================================
# A* PATH PLANNER
# ============================================================

def make_grid_bounds(a, b):
    pad = 0.025
    min_lat = min(a["lat"], b["lat"]) - pad
    max_lat = max(a["lat"], b["lat"]) + pad
    min_lon = min(a["lon"], b["lon"]) - pad
    max_lon = max(a["lon"], b["lon"]) + pad
    cell = 0.002
    return min_lat, max_lat, min_lon, max_lon, cell


def to_grid(lat, lon, min_lat, min_lon, cell):
    return int(round((lon - min_lon) / cell)), int(round((lat - min_lat) / cell))


def from_grid(x, y, min_lat, min_lon, cell):
    return min_lat + y * cell, min_lon + x * cell


def astar(start, goal):
    min_lat, max_lat, min_lon, max_lon, cell = make_grid_bounds(start, goal)
    s = to_grid(start["lat"], start["lon"], min_lat, min_lon, cell)
    g = to_grid(goal["lat"], goal["lon"], min_lat, min_lon, cell)

    # Simulated no-fly cells. They make A* visibly useful without
    # pretending these are authoritative aviation restrictions.
    obstacles = set()
    cx = int(round((s[0] + g[0]) / 2))
    cy = int(round((s[1] + g[1]) / 2))
    for dx in range(-2, 3):
        for dy in range(-2, 3):
            if (dx * dx + dy * dy) <= 4:
                obstacles.add((cx + dx, cy + dy))
    obstacles.discard(s)
    obstacles.discard(g)

    q = [(0.0, s)]
    came = {}
    gscore = {s: 0.0}
    visited = set()

    dirs = [
        (1, 0), (-1, 0), (0, 1), (0, -1),
        (1, 1), (1, -1), (-1, 1), (-1, -1)
    ]

    def h(p):
        return abs(p[0] - g[0]) + abs(p[1] - g[1])

    while q:
        _, cur = heapq.heappop(q)
        if cur in visited:
            continue
        visited.add(cur)

        if cur == g:
            path = [cur]
            while cur in came:
                cur = came[cur]
                path.append(cur)
            path.reverse()

            result = []
            for x, y in path:
                lat, lon = from_grid(x, y, min_lat, min_lon, cell)
                result.append({"lat": lat, "lon": lon})
            if result:
                result[0] = {"lat": start["lat"], "lon": start["lon"]}
                result[-1] = {"lat": goal["lat"], "lon": goal["lon"]}
            return result

        for dx, dy in dirs:
            nxt = (cur[0] + dx, cur[1] + dy)
            if nxt in obstacles:
                continue
            if nxt[0] < -20 or nxt[1] < -20:
                continue
            if nxt[0] > 500 or nxt[1] > 500:
                continue

            cost = math.sqrt(dx * dx + dy * dy)
            tentative = gscore[cur] + cost
            if tentative < gscore.get(nxt, float("inf")):
                came[nxt] = cur
                gscore[nxt] = tentative
                heapq.heappush(q, (tentative + h(nxt), nxt))

    return [
        {"lat": start["lat"], "lon": start["lon"]},
        {"lat": goal["lat"], "lon": goal["lon"]},
    ]


def route_distance(route):
    return sum(
        haversine(route[i-1]["lat"], route[i-1]["lon"],
                  route[i]["lat"], route[i]["lon"])
        for i in range(1, len(route))
    )


# ============================================================
# LIVE WEATHER
# ============================================================

def weather_label(code, rain, wind, gust, cloud, is_day=True):
    if code is None:
        return "Weather unavailable", "🌐"

    if code in (95, 96, 99):
        return "Thunderstorm", "⛈️"
    if rain > 2.0 or code in (61, 63, 65, 80, 81, 82):
        return "Rain", "🌧️"
    if wind >= 45 or gust >= 60:
        return "Severe Wind / Storm Risk", "⛈️"
    if wind >= 30 or gust >= 45:
        return "Strong Wind", "💨"
    if code in (1, 2, 3):
        return "Cloudy", "☁️"
    if code in (45, 48):
        return "Fog", "🌫️"
    if code in (51, 53, 55, 56, 57):
        return "Drizzle", "🌦️"
    return ("Sunny" if is_day else "Clear Night"), ("☀️" if is_day else "🌙")


def fetch_live_weather():
    global weather_state
    params = {
        "latitude": CITY_LAT,
        "longitude": CITY_LON,
        "current": ",".join([
            "temperature_2m",
            "relative_humidity_2m",
            "apparent_temperature",
            "precipitation",
            "rain",
            "weather_code",
            "cloud_cover",
            "wind_speed_10m",
            "wind_direction_10m",
            "wind_gusts_10m",
            "is_day",
        ]),
        "wind_speed_unit": "kmh",
        "temperature_unit": "celsius",
        "precipitation_unit": "mm",
        "timezone": "Asia/Kolkata",
    }

    url = "https://api.open-meteo.com/v1/forecast?" + urlencode(params)

    try:
        with urlopen(url, timeout=8) as response:
            data = json.loads(response.read().decode("utf-8"))

        cur = data.get("current", {})
        code = cur.get("weather_code")
        rain = float(cur.get("rain") or 0)
        wind = float(cur.get("wind_speed_10m") or 0)
        gust = float(cur.get("wind_gusts_10m") or 0)
        cloud = float(cur.get("cloud_cover") or 0)
        is_day = bool(cur.get("is_day", 1))
        condition, emoji = weather_label(code, rain, wind, gust, cloud, is_day)

        with lock:
            weather_state = {
                "ok": True,
                "condition": condition,
                "emoji": emoji,
                "temperature": cur.get("temperature_2m"),
                "apparent_temperature": cur.get("apparent_temperature"),
                "humidity": cur.get("relative_humidity_2m"),
                "rain": rain,
                "precipitation": float(cur.get("precipitation") or 0),
                "wind_speed": wind,
                "wind_direction": float(cur.get("wind_direction_10m") or 0),
                "wind_gusts": gust,
                "weather_code": code,
                "cloud_cover": cloud,
                "is_day": is_day,
                "updated_at": datetime.now().astimezone().isoformat(timespec="seconds"),
                "source": "Open-Meteo",
                "error": None,
            }
    except Exception as exc:
        with lock:
            weather_state["ok"] = False
            weather_state["condition"] = "Weather API unavailable"
            weather_state["emoji"] = "🌐"
            weather_state["error"] = str(exc)
            weather_state["updated_at"] = datetime.now().astimezone().isoformat(timespec="seconds")


def weather_loop():
    while True:
        fetch_live_weather()
        time.sleep(WEATHER_REFRESH_SECONDS)


threading.Thread(target=weather_loop, daemon=True).start()


# ============================================================
# WIND / DRONE ATTITUDE SIMULATION
# ============================================================

def wind_components(speed_kmh, direction_deg):
    # Meteorological direction means the direction FROM which wind comes.
    # Convert to a vector toward which it pushes the drone.
    push_dir = (direction_deg + 180) % 360
    rad = math.radians(push_dir)
    return speed_kmh * math.sin(rad), speed_kmh * math.cos(rad)


def wind_compensation(position, target, weather):
    desired = bearing(position, target)
    wind_speed = float(weather.get("wind_speed") or 0)
    wind_dir = float(weather.get("wind_direction") or 0)
    gust = float(weather.get("wind_gusts") or 0)

    # Wind direction is the direction FROM which wind arrives.
    wind_push = (wind_dir + 180) % 360
    rel = math.radians(angle_diff(wind_push, desired))

    crosswind = abs(wind_speed * math.sin(rel))
    headwind = wind_speed * math.cos(rel)

    # Simple vector compensation model for the application visualization.
    correction = math.degrees(math.atan2(crosswind, max(12.0, 60.0)))
    sign = 1 if angle_diff(wind_push, desired) >= 0 else -1
    compensated_heading = (desired - sign * correction) % 360

    # Simulated attitude response. This is not a certified flight controller.
    roll = clamp(sign * correction * 0.65, -18, 18)
    pitch = clamp((-headwind / 45.0) * 7.0, -10, 10)

    severity = "NORMAL"
    if gust >= 60 or wind_speed >= 45:
        severity = "SEVERE"
    elif gust >= 45 or wind_speed >= 30:
        severity = "STRONG"
    elif wind_speed >= 20:
        severity = "MODERATE"

    return {
        "desired_heading": round(desired, 1),
        "wind_direction": round(wind_dir, 1),
        "wind_push_direction": round(wind_push, 1),
        "wind_speed": round(wind_speed, 1),
        "wind_gusts": round(gust, 1),
        "crosswind": round(crosswind, 1),
        "headwind": round(headwind, 1),
        "correction_angle": round(correction, 1),
        "compensated_heading": round(compensated_heading, 1),
        "roll_angle": round(roll, 1),
        "pitch_angle": round(pitch, 1),
        "yaw_angle": round(compensated_heading, 1),
        "severity": severity,
        "message": (
            "Severe wind: hold/return recommended"
            if severity == "SEVERE"
            else "Strong wind: active compensation"
            if severity == "STRONG"
            else "Moderate wind: compensation active"
            if severity == "MODERATE"
            else "Normal flight"
        ),
    }


# ============================================================
# MISSION HELPERS
# ============================================================

def find_customer(customer_id):
    return next((x for x in CUSTOMERS if x["id"] == customer_id), None)


def find_pickup(parcel_type, pickup_id):
    return next((x for x in PICKUPS.get(parcel_type, []) if x["id"] == pickup_id), None)


def nearest_charger(position):
    return min(CHARGERS, key=lambda x: dist(position, x))


def nearest_office(position):
    return min(PARCEL_OFFICES, key=lambda x: dist(position, x))


def available_drone(parcel_type, payload_kg=0.0, required_range_km=0.0):
    """Autonomously select a drone that can safely carry the order.
    Payload is checked before battery/range so an overloaded drone is never assigned.
    """
    payload_kg = max(0.0, float(payload_kg or 0))
    required_range_km = max(0.0, float(required_range_km or 0))
    candidates = []
    for d in DRONES.values():
        if d["type"] != parcel_type or d["busy"] or d["battery"] < 25:
            continue
        capacity = float(d.get("max_payload_kg", 0))
        if payload_kg > capacity:
            continue
        # Keep extra reserve for heavy payloads and longer missions.
        load_ratio = payload_kg / max(capacity, 0.1)
        battery_need = min(55, required_range_km * 1.8 + load_ratio * 18)
        if d["battery"] < 25 + battery_need * 0.35:
            continue
        candidates.append(d)
    if not candidates:
        return None
    # Best balance: enough payload margin, then battery, then health.
    return max(candidates, key=lambda x: (
        float(x.get("max_payload_kg",0)) - payload_kg,
        x.get("battery",0),
        x.get("health",0)
    ))


def status_text(phase):
    return {
        "pickup": "GOING_TO_PICKUP",
        "customer": "GOING_TO_CUSTOMER",
        "return": "RETURNING_TO_BASE",
    }.get(phase, "UNKNOWN")


def save_history(m):
    conn = db_connect()
    conn.execute("""
        INSERT OR REPLACE INTO deliveries
        (id,drone_id,parcel_type,item,pickup_name,pickup_lat,pickup_lon,customer_id,customer_name,customer_lat,customer_lon,weather_condition,wind_speed,wind_direction,max_gust,total_distance,delivery_time_seconds,status,created_at,completed_at,avg_speed,weather_changes,obstacle_events)
        VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?)
    """, (
        m["id"], m["drone_id"], m["parcel_type"], m["item"],
        m["pickup"]["name"], m["pickup"]["lat"], m["pickup"]["lon"],
        m["destination"]["id"], m["destination"]["name"],
        m["destination"]["lat"], m["destination"]["lon"],
        m["weather_at_creation"]["condition"],
        m["weather_at_creation"]["wind_speed"],
        m["weather_at_creation"]["wind_direction"],
        m["weather_at_creation"]["wind_gusts"],
        m["total_distance"],
        max(0, time.time() - m["created_timestamp"]),
        m["final_status"],
        datetime.fromtimestamp(m["created_timestamp"]).astimezone().isoformat(timespec="seconds"),
        datetime.now().astimezone().isoformat(timespec="seconds"),
        m.get("avg_speed",0), m.get("weather_changes",0), m.get("obstacle_events",0),
    ))
    conn.commit()
    conn.close()


def mission_dict(m):
    # Never expose the actual OTP/QR secret through the fleet tracking API.
    x = dict(m)
    x.pop("delivery_otp", None)
    x.pop("qr_payload", None)
    x.pop("qr_image", None)
    return x


def normalize_phone(phone):
    """Return a basic E.164-style phone number for SMS providers."""
    phone = (phone or "").strip().replace(" ", "").replace("-", "")
    if phone.startswith("0"):
        phone = "+91" + phone[1:]
    elif phone.isdigit() and len(phone) == 10:
        phone = "+91" + phone
    return phone


def make_delivery_qr(mission_id, otp):
    """Create a QR image containing a delivery verification token."""
    payload = f"SKYDELIVER|{mission_id}|{otp}"
    if qrcode is None:
        return None
    img = qrcode.make(payload)
    buf = BytesIO()
    img.save(buf, format="PNG")
    return "data:image/png;base64," + base64.b64encode(buf.getvalue()).decode("ascii")


def send_customer_sms(phone, mission_id, otp):
    """Optional real SMS through Twilio REST API. Falls back to demo mode if not configured."""
    sid = os.getenv("TWILIO_ACCOUNT_SID")
    token = os.getenv("TWILIO_AUTH_TOKEN")
    from_number = os.getenv("TWILIO_FROM_NUMBER")
    phone = normalize_phone(phone)
    if not phone:
        return {"sent": False, "mode": "demo", "message": "No customer phone number supplied."}
    if not (sid and token and from_number):
        return {"sent": False, "mode": "demo", "message": "Twilio not configured; OTP is available in the Customer Verification panel."}

    import urllib.request
    import urllib.parse
    import base64 as _b64
    endpoint = f"https://api.twilio.com/2010-04-01/Accounts/{sid}/Messages.json"
    body = urllib.parse.urlencode({
        "To": phone,
        "From": from_number,
        "Body": f"SkyDeliver: Your delivery OTP is {otp}. Give this OTP to the drone delivery agent only when receiving order {mission_id}."
    }).encode("utf-8")
    auth = _b64.b64encode(f"{sid}:{token}".encode()).decode()
    req = urllib.request.Request(endpoint, data=body, method="POST")
    req.add_header("Authorization", "Basic " + auth)
    try:
        with urllib.request.urlopen(req, timeout=10) as response:
            response.read()
        return {"sent": True, "mode": "sms", "message": f"OTP sent to {phone[-4:].rjust(len(phone), '*')}"}
    except Exception as exc:
        return {"sent": False, "mode": "demo", "message": f"SMS failed; use the Customer Verification panel. ({exc})"}


def point_in_nofly(point):
    for z in NO_FLY_ZONES:
        if haversine(point["lat"],point["lon"],z["lat"],z["lon"]) <= z["radius_km"]:
            return z
    return None

def push_alert(level,title,message,mission_id=None):
    system_alerts.insert(0,{"time":datetime.now().astimezone().isoformat(timespec="seconds"),"level":level,"title":title,"message":message,"mission_id":mission_id})
    del system_alerts[30:]

def obstacle_position(position, target, step_count):
    """Simulated airborne obstacle sensor: birds, balloons, kites or another drone.
    It is deliberately intermittent so the demo visibly shows detection and avoidance.
    """
    # Deterministic-ish event every few ticks plus random probability.
    if step_count > 0 and step_count % 13 == 0 or random.random() < 0.025:
        heading = bearing(position, target)
        side = random.choice([-1, 1])
        distance_m = random.randint(35, 90)
        lat_scale = math.cos(math.radians(position["lat"]))
        dlat = math.cos(math.radians(heading)) * distance_m / 111000.0
        dlon = math.sin(math.radians(heading)) * distance_m / max(111000.0 * lat_scale, 1)
        # Put obstacle slightly across the current path, not directly at the drone.
        perp = math.radians(heading + 90 * side)
        lat = position["lat"] + dlat + math.cos(perp) * 18 / 111000.0
        lon = position["lon"] + dlon + math.sin(perp) * 18 / max(111000.0 * lat_scale, 1)
        kind = random.choice(["Bird Flock", "Large Bird", "Kite/Balloon", "Airborne Object"])
        return {"type": kind, "lat": lat, "lon": lon, "distance_m": distance_m, "bearing": (heading + random.uniform(-25,25)) % 360, "side": side}
    return None

def avoidance_waypoint(position, obstacle, target):
    side = obstacle.get("side", 1)
    base_heading = bearing(position, target)
    avoid_heading = (base_heading + 70 * side) % 360
    r = math.radians(avoid_heading)
    meters = 90
    lat = position["lat"] + math.cos(r) * meters / 111000.0
    lon = position["lon"] + math.sin(r) * meters / max(111000.0 * math.cos(math.radians(position["lat"])),1)
    return {"lat":lat,"lon":lon}


def create_mission(parcel_type, pickup_id, customer_id, recipient_available=True, customer_phone="", customer_override=None, payload_kg=0.0):
    pickup = find_pickup(parcel_type, pickup_id)
    customer = customer_override if customer_override else find_customer(customer_id)
    if not pickup or not customer:
        return None, "Invalid pickup or customer point."

    # Select a real candidate first, then build routes from THAT drone's base.
    # This fixes the dispatch bug caused by using `drone` before assignment.
    payload_kg = max(0.1, float(payload_kg or 0.0))
    candidates = [
        d for d in DRONES.values()
        if d["type"] == parcel_type
        and not d["busy"]
        and d["battery"] >= 25
        and payload_kg <= float(d.get("max_payload_kg", 0))
    ]
    if not candidates:
        max_cap = max([d.get("max_payload_kg",0) for d in DRONES.values() if d["type"] == parcel_type] or [0])
        return None, f"No suitable drone available for {payload_kg:.1f} kg payload. Required capacity: {payload_kg:.1f} kg; maximum {parcel_type} drone capacity: {max_cap:.1f} kg."

    drone = max(candidates, key=lambda d: (
        float(d.get("max_payload_kg", 0)) - payload_kg,
        float(d.get("battery", 0)),
        float(d.get("health", 0))
    ))
    base = BASES[drone["base"]]
    r1 = astar(base, pickup)
    r2 = astar(pickup, customer)
    r3 = astar(customer, base)

    if not r1 or not r2 or not r3:
        return None, "No safe A* route is available for this delivery."

    total = route_distance(r1) + route_distance(r2) + route_distance(r3)
    required_battery = min(55, total * 1.8 + (payload_kg / max(float(drone.get("max_payload_kg", 0)), 0.1)) * 18)
    if drone["battery"] < 25 + required_battery * 0.35:
        return None, f"{drone['id']} does not have enough battery reserve for this {total:.1f} km mission."
    mission_id = "SD-" + uuid.uuid4().hex[:8].upper()
    delivery_otp = f"{secrets.randbelow(1000000):06d}"
    customer_phone = normalize_phone(customer_phone)
    qr_image = make_delivery_qr(mission_id, delivery_otp)

    m = {
        "id": mission_id,
        "drone_id": drone["id"],
        "parcel_type": parcel_type,
        "item": pickup["item"],
        "priority": "NORMAL",
        "payload_kg": round(payload_kg,2),
        "drone_max_payload_kg": float(drone.get("max_payload_kg",0)),
        "payload_load_percent": round((payload_kg / max(float(drone.get("max_payload_kg",0)),0.1))*100,1),
        "payload_balance_status": "BALANCED" if payload_kg <= float(drone.get("max_payload_kg",0))*0.85 else "HIGH LOAD - AUTONOMOUS BALANCE",
        "payload_margin_kg": round(float(drone.get("max_payload_kg",0))-payload_kg,2),
        "dynamic_reroutes": 0,
        "emergency": False,
        "emergency_reason": None,
        "gps_signal": drone.get("gps_signal",95),
        "connection": drone.get("connection",95),
        "temperature": drone.get("temperature",32),
        "flight_altitude": drone.get("altitude",65),
        "collision_status": "CLEAR",
        "route_reason": "Initial A* route",
        "obstacle_event": None,
        "obstacle_events": 0,
        "obstacle_clearance_m": 0,
        "weather_changes": 0,
        "last_weather_signature": None,
        "last_weather_condition": None,
        "weather_zone": "NORMAL HYDERABAD WEATHER",
        "weather_alert": "🟢 Normal local weather corridor",
        "speed_reason": "Normal conditions",
        "speed_history": [],
        "step_count": 0,
        "avg_speed": 0.0,
        "pickup": pickup,
        "destination": customer,
        "base": base,
        "recipient_available": bool(recipient_available),
        "customer_phone": customer_phone,
        "delivery_otp": delivery_otp,
        "qr_payload": f"SKYDELIVER|{mission_id}|{delivery_otp}",
        "qr_image": qr_image,
        "verification_status": "PENDING",
        "verification_attempts": 0,
        "otp_sent": False,
        "otp_sent_at": None,
        "otp_expires_at": None,
        "otp_valid_seconds": 60,
        "otp_remaining_seconds": None,
        "availability_mode": "OTP_60_SECOND_AUTO",
        "sms_result": {},
        "phase": "pickup",
        "status": "GOING_TO_PICKUP",
        "position": dict(base),
        "route": r1,
        "route_to_pickup": r1,
        "route_to_customer": r2,
        "route_to_base": r3,
        "route_index": 0,
        "battery": drone["battery"],
        "speed": drone["max_speed"],
        "eta_minutes": 0,
        "distance_completed": 0.0,
        "total_distance": round(total, 3),
        "pickup_wait": 0,
        "customer_wait": 0,
        "charging": False,
        "parcel_office": None,
        "created_timestamp": time.time(),
        "weather_at_creation": dict(weather_state),
        "final_status": None,
        "message": f"{drone['id']} departing {base['name']} for {pickup['name']}.",
        "attitude": {},
    }

    drone["busy"] = True
    drone["mission_id"] = mission_id
    missions[mission_id] = m
    return m, None


# ============================================================
# MISSION SIMULATION
# ============================================================

def complete_and_return(m, message):
    m["status"] = "DELIVERED"
    m["final_status"] = "DELIVERED"
    m["message"] = message
    m["phase"] = "return"
    m["route"] = m["route_to_base"]
    m["route_index"] = 0
    m["position"] = dict(m["destination"])


def update_mission(m):
    # Weather is recalculated from the drone's CURRENT coordinate every tick.
    # This creates clearly different rain/storm/clear corridors across Hyderabad.
    weather = local_weather_for_position(m["position"], weather_state, m.get("step_count",0) + 1)
    m["mission_weather"] = weather
    previous_condition = m.get("last_weather_condition")
    if previous_condition and previous_condition != weather.get("condition"):
        msg = f"🌦️ WEATHER CHANGE: {previous_condition} → {weather.get('condition')} at {weather.get('zone')}"
        m["message"] = msg
        push_alert("warning","Weather corridor changed",msg,m["id"])
    m["last_weather_condition"] = weather.get("condition")
    drone = DRONES[m["drone_id"]]
    m["step_count"] = m.get("step_count",0) + 1

    # Track changing weather during the mission, not just at dispatch time.
    weather_signature = (weather.get("condition"), round(float(weather.get("wind_speed") or 0),1), round(float(weather.get("rain") or 0),1))
    if m.get("last_weather_signature") is not None and weather_signature != m["last_weather_signature"]:
        m["weather_changes"] = m.get("weather_changes",0) + 1
    m["last_weather_signature"] = weather_signature

    # Simulated live avionics telemetry
    drone["gps_signal"] = max(35,min(100,drone.get("gps_signal",95)+random.uniform(-1.2,1.2)))
    drone["connection"] = max(30,min(100,drone.get("connection",95)+random.uniform(-1.5,1.5)))
    drone["temperature"] = max(25,min(65,drone.get("temperature",32)+random.uniform(-0.8,0.8)))
    m["gps_signal"]=round(drone["gps_signal"],1)
    m["connection"]=round(drone["connection"],1)
    m["temperature"]=round(drone["temperature"],1)
    m["gps_accuracy_m"]=round(max(2,8-drone["gps_signal"]/18),1)
    m["connection_latency_ms"]=int(max(25,110-drone["connection"]*.7))
    m["maintenance_alert"]="⚠ Predictive maintenance recommended" if drone.get("health",95)<70 else None

    if m.get("emergency") and m["phase"] != "return":
        m["phase"]="return"; m["route"]=astar(m["position"],m["base"]); m["route_index"]=0
        m["status"]="EMERGENCY_RETURN"; m["speed"]=0
        m["message"]=f"🚨 Emergency return: {m.get('emergency_reason','operator command')}"
        return

    if m["phase"] != "return" and (m["gps_signal"]<45 or m["connection"]<40):
        m["status"]="SAFETY_HOLD"; m["speed"]=0
        m["message"]="📡 Weak GPS/communication signal. Holding position."
        return

    # Severe weather safety behaviour:
    # The app shows the attitude needed to counter wind. If conditions
    # become severe, it pauses forward travel and changes mission status.
    if weather["ok"]:
        attitude_target = m["destination"] if m["phase"] == "customer" else (
            m["pickup"] if m["phase"] == "pickup" else m["base"]
        )
        m["attitude"] = wind_compensation(m["position"], attitude_target, weather)

        # Weather is telemetry only; it does not stop or slow the demo flight.\n    else:
        m["attitude"] = {}

    # Pickup waiting
    if m["status"] == "ARRIVED_AT_PICKUP":
        m["pickup_wait"] += 1
        m["speed"] = 0
        if m["pickup_wait"] >= 3:
            m["status"] = "PACKAGE_PICKED_UP"
            m["message"] = f"📦 Package picked up at {m['pickup']['name']}."
        return

    if m["status"] == "PACKAGE_PICKED_UP":
        m["phase"] = "customer"
        m["route"] = m["route_to_customer"]
        m["route_index"] = 0
        m["position"] = dict(m["pickup"])
        m["status"] = "GOING_TO_CUSTOMER"
        m["message"] = f"🚁 Delivering {m['item']} to {m['destination']['name']}."
        return

    # Customer waiting
    if m["status"] == "ARRIVED_AT_DESTINATION":
        m["customer_wait"] += 1
        m["speed"] = 0
        if m["customer_wait"] >= 3:
            if m["recipient_available"]:
                if m.get("verification_status") == "VERIFIED":
                    m["status"] = "DELIVERED"
                    m["final_status"] = "DELIVERED"
                    m["message"] = "✅ Customer verified by OTP/QR. Package handed over successfully. Returning to base."
                else:
                    m["status"] = "WAITING_FOR_OTP"
                    m["message"] = "🔐 Customer identity verification required. Ask the customer for the OTP or scan their QR code."
            else:
                office = nearest_office(m["destination"])
                m["parcel_office"] = office
                m["status"] = "RECIPIENT_UNAVAILABLE"
                m["final_status"] = "RECIPIENT_UNAVAILABLE"
                m["message"] = f"⚠️ Recipient unavailable. Package routed to {office['name']}."
        return

    if m["status"] == "WAITING_FOR_OTP":
        m["speed"] = 0
        expires = m.get("otp_expires_at")
        if expires:
            remaining = max(0, int(expires - time.time()))
            m["otp_remaining_seconds"] = remaining
            if remaining <= 0:
                office = nearest_office(m["destination"])
                m["parcel_office"] = office
                m["recipient_available"] = False
                m["status"] = "RECIPIENT_UNAVAILABLE"
                m["final_status"] = "RECIPIENT_UNAVAILABLE"
                m["message"] = f"⏱️ OTP expired after 60 seconds. Recipient marked unavailable. Package routed to {office['name']}."
                push_alert("warning","Customer unavailable",m["message"],m["id"])
                return
        m["message"] = f"🔐 Waiting for customer OTP/QR verification. {m.get('otp_remaining_seconds',60)} seconds remaining."
        return

    if m["status"] in ("DELIVERED", "RECIPIENT_UNAVAILABLE"):
        m["phase"] = "return"
        m["route"] = m["route_to_base"]
        m["route_index"] = 0
        m["position"] = dict(m["destination"])
        m["status"] = "RETURNING_TO_BASE"
        m["message"] = "🔋 Delivery complete. Drone returning to base."
        return

    if m["status"] == "SAFETY_HOLD":
        if m.get("gps_signal",0)>=55 and m.get("connection",0)>=50:
            m["status"]=status_text(m["phase"]); m["message"]="📡 Link recovered. Resuming flight."
        else:
            return

    if m["status"] == "WEATHER_HOLD":
        # Resume when the current weather is no longer severe.
        if not weather["ok"] or (weather["wind_speed"] < 45 and weather["wind_gusts"] < 60):
            m["status"] = status_text(m["phase"])
            m["message"] = "Weather improved. Resuming autonomous flight."
        else:
            return

    # Bird/airborne obstacle radar detection and lateral avoidance.
    if m["phase"] != "return" and not m.get("obstacle_event"):
        target = m["pickup"] if m["phase"] == "pickup" else m["destination"]
        obs = obstacle_position(m["position"], target, m.get("step_count",0))
        if obs:
            m["obstacle_event"] = obs
            m["obstacle_events"] = m.get("obstacle_events",0) + 1
            m["obstacle_clearance_m"] = obs["distance_m"]
            m["status"] = "OBSTACLE_DETECTED"
            m["speed"] = 0
            m["message"] = f"🦅 Radar detected {obs['type']} at {obs['distance_m']} m. Drone holding and moving aside."
            push_alert("warning","Airborne obstacle detected",m["message"],m["id"])
            # Build a short lateral avoidance waypoint then continue toward target.
            wp = avoidance_waypoint(m["position"], obs, target)
            new_route = astar(m["position"], wp)[:-1] + astar(wp, target)
            m["route"] = new_route
            m["route_index"] = 0
            m["route_reason"] = f"Radar avoidance: {obs['type']}"
            return

    if m.get("obstacle_event"):
        # Once the drone has moved beyond the avoidance manoeuvre, clear the event.
        if m.get("route_index",0) >= min(4, len(m.get("route",[]))-1):
            m["obstacle_event"] = None
            m["status"] = status_text(m["phase"])
            m["message"] = "🟢 Radar path clear. Resuming planned route."

    # Multi-drone separation
    rank={"NORMAL":1,"HIGH":2,"URGENT":3,"CRITICAL":4}
    for other in missions.values():
        if other["id"]==m["id"] or other["status"] in ("CHARGING","AVAILABLE"):
            continue
        if haversine(m["position"]["lat"],m["position"]["lon"],other["position"]["lat"],other["position"]["lon"])<0.12 and rank.get(m.get("priority","NORMAL"),1)<rank.get(other.get("priority","NORMAL"),1):
            m["collision_status"]=f"YIELDING TO {other['drone_id']}"; m["speed"]=0; m["message"]=f"🛑 Airspace separation: yielding to {other['drone_id']}."; return
    m["collision_status"]="CLEAR"

    route = m["route"]
    if not route:
        return

    if m["route_index"] >= len(route) - 1:
        if m["phase"] == "pickup":
            m["position"] = dict(m["pickup"])
            m["status"] = "ARRIVED_AT_PICKUP"
            m["speed"] = 0
            m["message"] = f"📍 Arrived at {m['pickup']['name']}. Waiting for package."
            return

        if m["phase"] == "customer":
            m["position"] = dict(m["destination"])
            m["status"] = "ARRIVED_AT_DESTINATION"
            m["speed"] = 0
            if not m.get("otp_sent"):
                m["sms_result"] = send_customer_sms(m.get("customer_phone", ""), m["id"], m["delivery_otp"])
                m["otp_sent"] = True
                m["otp_sent_at"] = time.time()
                m["otp_expires_at"] = time.time() + 60
                m["otp_remaining_seconds"] = 60
            m["message"] = "📍 Arrived at customer delivery point. OTP sent. Customer has 60 seconds to verify."
            return

        if m["phase"] == "return":
            m["position"] = dict(m["base"])
            m["status"] = "CHARGING"
            m["charging"] = True
            m["speed"] = 0
            m["message"] = "⚡ Returned to base. Automatic charging started."
            return

    old = dict(m["position"])
    nxt = route[m["route_index"] + 1]
    segment_km = haversine(old["lat"], old["lon"], nxt["lat"], nxt["lon"])

    m["position"] = dict(nxt)
    m["route_index"] += 1
    m["distance_completed"] += segment_km

    # Dynamic speed: speed depends on remaining distance AND live weather.
    weather_factor = 1.0
    battery_factor = 1.0
    ws = float(weather.get("wind_speed") or 0) if weather.get("ok") else 0.0
    rain = float(weather.get("rain") or 0) if weather.get("ok") else 0.0
    gust = float(weather.get("wind_gusts") or 0) if weather.get("ok") else 0.0

    # Distance factor: farther from destination -> cruise faster; close to destination -> slow down.
    remaining_before = route_distance(route[m["route_index"]:])
    distance_factor = clamp(0.62 + min(remaining_before / 5.0, 0.38), 0.62, 1.0)

    condition = str(weather.get("condition") or "").upper()
    if "THUNDERSTORM" in condition or "STORM" in condition:
        weather_factor *= 0.48; battery_factor *= 1.70
        m["speed_reason"] = "⛈️ Storm corridor: reduced speed + high energy demand"
    elif "HEAVY RAIN" in condition:
        weather_factor *= 0.62; battery_factor *= 1.45
        m["speed_reason"] = "🌧️ Heavy-rain corridor: reduced speed"
    elif "RAIN" in condition:
        weather_factor *= 0.78; battery_factor *= 1.25
        m["speed_reason"] = "🌦️ Rain corridor: reduced speed"
    elif ws >= 35 or gust >= 50:
        weather_factor *= 0.68; battery_factor *= 1.40
        m["speed_reason"] = "💨 Strong-wind corridor: compensation active"
    elif ws >= 20:
        weather_factor *= 0.84; battery_factor *= 1.15
        m["speed_reason"] = "💨 Moderate wind compensation"
    else:
        m["speed_reason"] = "🟢 Clear/normal weather corridor"
    if rain > 0:
        weather_factor *= max(0.72, 1.0 - min(rain/10.0,0.28))
        battery_factor *= 1.0 + min(rain/20.0,0.25)
        m["speed_reason"] += " + rain adjustment"
    # Requested demo behavior: sunny/clear corridor, so weather never slows the drone.
    if DEMO_CLEAR_WEATHER:
        weather_factor = 1.0
        battery_factor = 1.0
        m["speed_reason"] = "☀️ Sunny / clear weather • normal cruise"

    m["weather_zone"] = weather.get("zone")
    m["weather_alert"] = weather.get("weather_alert")

    # Crosswind compensation increases energy slightly.
    compensation = abs(m.get("attitude", {}).get("roll_angle", 0))
    battery_factor *= 1.0 + compensation / 100.0

    m["speed"] = round(drone["max_speed"] * weather_factor * distance_factor, 1)
    m["speed_history"] = (m.get("speed_history",[]) + [m["speed"]])[-60:]
    m["avg_speed"] = round(sum(m["speed_history"])/len(m["speed_history"]),1)
    m["battery"] = max(0.0, m["battery"] - segment_km * 0.55 * battery_factor)

    remaining = route_distance(route[m["route_index"]:])
    m["eta_minutes"] = round((remaining / max(m["speed"], 5)) * 60, 1)

    if m["battery"] <= 12 and m["phase"] != "return":
        m["emergency"]=True; m["emergency_reason"]="Critical battery"
        m["phase"]="return"; m["route"]=astar(m["position"],m["base"]); m["route_index"]=0
        m["status"]="EMERGENCY_RETURN"; m["speed"]=0
        m["message"]="🔋 Critical battery. Automatic return-to-base initiated."
        push_alert("danger","Critical battery",m["message"],m["id"])
        return

    if m["status"] == "LOW_BATTERY":
        if m["battery"] > 15:
            m["status"] = status_text(m["phase"])

    if m["phase"] == "pickup":
        m["status"] = "GOING_TO_PICKUP"
        m["message"] = "🚁 Flying to pickup location."
    elif m["phase"] == "customer":
        m["status"] = "GOING_TO_CUSTOMER"
        m["message"] = "🚁 Delivering package to customer."
    else:
        m["status"] = "RETURNING_TO_BASE"
        m["message"] = "🚁 Returning to drone base."


def update_charging(m):
    if m["status"] != "CHARGING":
        return

    m["speed"] = 0
    m["battery"] = min(100.0, m["battery"] + 8.0)

    if m["battery"] >= 100:
        m["battery"] = 100.0
        m["charging"] = False
        m["status"] = "AVAILABLE"
        m["message"] = "🟢 Charging complete. Drone available."
        d = DRONES[m["drone_id"]]
        d["busy"] = False
        d["mission_id"] = None
        if m["final_status"] is None:
            m["final_status"] = "DELIVERED"
        save_history(m)


def simulation_loop():
    while True:
        with lock:
            for m in list(missions.values()):
                try:
                    if m["status"] == "CHARGING":
                        update_charging(m)
                    elif m["status"] not in ("AVAILABLE",):
                        update_mission(m)
                except Exception as exc:
                    m["message"] = f"Simulation error: {exc}"
        time.sleep(SIMULATION_TICK_SECONDS)


threading.Thread(target=simulation_loop, daemon=True).start()


# ============================================================
# API
# ============================================================

@app.route("/")
def index():
    return render_template_string(HTML)


@app.route("/api/data")
def api_data():
    return jsonify({
        "drones": list(DRONES.values()),
        "bases": list(BASES.values()),
        "pickups": PICKUPS,
        "customers": CUSTOMERS,
        "chargers": CHARGERS,
        "parcel_offices": PARCEL_OFFICES,
        "no_fly_zones": NO_FLY_ZONES,
        "customer_count": len(CUSTOMERS),
        "zones": {z: v[2] for z, v in ZONES.items()},
    })


@app.route("/api/weather")
def api_weather():
    with lock:
        return jsonify(weather_state)


@app.route("/api/start", methods=["POST"])
def api_start():
    data = request.get_json(silent=True) or {}
    parcel_type = data.get("parcel_type", "food")
    pickup_id = data.get("pickup_id")
    customer_id = data.get("customer_id")
    # Recipient availability is decided automatically by the OTP timer.
    recipient_available = True
    try:
        payload_kg = max(0.1, float(data.get("payload_kg", 1.0) or 1.0))
    except (TypeError, ValueError):
        payload_kg = 1.0
    customer_phone = normalize_phone(data.get("customer_phone", ""))
    customer_override = None
    if data.get("customer_lat") is not None and data.get("customer_lon") is not None:
        try:
            lat = float(data.get("customer_lat")); lon = float(data.get("customer_lon"))
            if not (16.9 <= lat <= 17.8 and 77.9 <= lon <= 79.0):
                return jsonify({"success": False, "message": "Please select a location inside Hyderabad."}), 400
            customer_override = {
                "id": "ADDR-" + uuid.uuid4().hex[:8].upper(),
                "zone": "Address Search",
                "name": str(data.get("customer_name") or "Selected Hyderabad Address"),
                "lat": lat, "lon": lon,
                "address": str(data.get("customer_address") or "Hyderabad")
            }
            customer_id = customer_override["id"]
        except (TypeError, ValueError):
            return jsonify({"success": False, "message": "Invalid address coordinates."}), 400

    if parcel_type not in PICKUPS:
        return jsonify({"success": False, "message": "Invalid parcel type."}), 400

    if not pickup_id:
        pickup_id = PICKUPS[parcel_type][0]["id"]

    if not customer_id:
        customer_id = CUSTOMERS[0]["id"]

    with lock:
        mission, error = create_mission(
            parcel_type,
            pickup_id,
            customer_id,
            recipient_available,
            customer_phone,
            customer_override=customer_override,
            payload_kg=payload_kg,
        )

    if error:
        return jsonify({"success": False, "message": error}), 400

    safe = mission_dict(mission)
    return jsonify({"success": True, "mission": safe, "customer_verification": {"mission_id": mission["id"], "phone": mission["customer_phone"], "message": "OTP will be sent when the drone reaches the customer."}})


@app.route("/api/geocode")
def api_geocode():
    """Free Hyderabad address search using OpenStreetMap Nominatim; no Google API key."""
    q = (request.args.get("q") or "").strip()
    if not q:
        return jsonify({"success": False, "message": "Enter a Hyderabad address."}), 400
    if "hyderabad" not in q.lower():
        q += ", Hyderabad, Telangana, India"
    try:
        url = "https://nominatim.openstreetmap.org/search?" + urlencode({
            "q": q, "format": "jsonv2", "limit": 5, "addressdetails": 1,
            "countrycodes": "in", "viewbox": "77.9,17.6,78.65,17.2", "bounded": 1
        })
        req = __import__('urllib.request', fromlist=['Request']).Request(
            url, headers={"User-Agent": "SkyDeliver-Academic-Project/1.0 (address search)"}
        )
        with urlopen(req, timeout=12) as resp:
            results = json.loads(resp.read().decode("utf-8"))
        clean=[]
        for r in results:
            lat=float(r["lat"]); lon=float(r["lon"])
            if 16.9 <= lat <= 17.8 and 77.9 <= lon <= 79.0:
                clean.append({
                    "lat":lat,"lon":lon,"display_name":r.get("display_name","Hyderabad"),
                    "type":r.get("type","location"), "importance":r.get("importance",0)
                })
        return jsonify({"success": bool(clean), "results": clean, "message": None if clean else "Address not found in Hyderabad. Try adding landmark, area, road number or PIN code."})
    except Exception as exc:
        return jsonify({"success": False, "message": "Address service unavailable. Check your internet connection and try again.", "error": str(exc)}), 503


@app.route("/api/verify", methods=["POST"])
def api_verify():
    data = request.get_json(silent=True) or {}
    mission_id = str(data.get("mission_id", "")).strip()
    otp = str(data.get("otp", "")).strip()
    qr_payload = str(data.get("qr_payload", "")).strip()

    with lock:
        m = missions.get(mission_id)
        if not m:
            return jsonify({"success": False, "message": "Delivery mission not found."}), 404
        if m["status"] not in ("ARRIVED_AT_DESTINATION", "WAITING_FOR_OTP"):
            return jsonify({"success": False, "message": "Drone is not currently waiting for customer verification."}), 400

        if m.get("otp_expires_at") and time.time() > m["otp_expires_at"]:
            office = nearest_office(m["destination"])
            m["parcel_office"] = office
            m["recipient_available"] = False
            m["status"] = "RECIPIENT_UNAVAILABLE"
            m["final_status"] = "RECIPIENT_UNAVAILABLE"
            m["otp_remaining_seconds"] = 0
            m["message"] = f"⏱️ OTP expired. Customer is marked unavailable and package is routed to {office['name']}."
            push_alert("warning","OTP expired",m["message"],m["id"])
            return jsonify({"success": False, "message": "OTP expired after 60 seconds. Recipient marked unavailable and package routed to the parcel office."}), 400

        m["otp_remaining_seconds"] = max(0, int(m.get("otp_expires_at", time.time()) - time.time()))
        supplied_otp = otp
        if qr_payload.startswith("SKYDELIVER|"):
            parts = qr_payload.split("|")
            if len(parts) == 3 and parts[1] == mission_id:
                supplied_otp = parts[2]

        m["verification_attempts"] += 1
        if secrets.compare_digest(supplied_otp, m["delivery_otp"]):
            m["verification_status"] = "VERIFIED"
            m["status"] = "DELIVERED"
            m["final_status"] = "DELIVERED"
            m["message"] = "✅ Customer verified by OTP/QR. Package handed over successfully. Returning to base."
            return jsonify({"success": True, "message": "Customer verified successfully. Package can be handed over.", "mission": mission_dict(m)})

        return jsonify({"success": False, "message": "Incorrect OTP/QR. Ask the customer to provide the correct code."}), 400


@app.route("/api/customer/<mission_id>")
def customer_view(mission_id):
    with lock:
        m = missions.get(mission_id)
        if not m:
            return jsonify({"success": False, "message": "Delivery not found."}), 404
        return jsonify({
            "success": True,
            "mission_id": m["id"],
            "customer": m["destination"],
            "item": m["item"],
            "otp": m["delivery_otp"],
            "qr_image": m["qr_image"],
            "verification_status": m["verification_status"],
            "status": m["status"],
            "otp_remaining_seconds": max(0, int(m.get("otp_expires_at", time.time()) - time.time())) if m.get("otp_expires_at") else None,
            "availability_mode": m.get("availability_mode"),
            "phone": m["customer_phone"],
        })


@app.route("/api/emergency/<mission_id>", methods=["POST"])
def api_emergency(mission_id):
    data=request.get_json(silent=True) or {}
    reason=str(data.get("reason","Operator safety command"))
    with lock:
        m=missions.get(mission_id)
        if not m: return jsonify({"success":False,"message":"Mission not found."}),404
        m["emergency"]=True; m["emergency_reason"]=reason
        m["phase"]="return"; m["route"]=astar(m["position"],m["base"]); m["route_index"]=0
        m["status"]="EMERGENCY_RETURN"; m["message"]=f"🚨 Emergency return initiated: {reason}"
        push_alert("danger","Emergency return",m["message"],mission_id)
        return jsonify({"success":True,"mission":mission_dict(m)})

@app.route("/api/reroute/<mission_id>", methods=["POST"])
def api_reroute(mission_id):
    with lock:
        m=missions.get(mission_id)
        if not m: return jsonify({"success":False,"message":"Mission not found."}),404
        target=m["pickup"] if m["phase"]=="pickup" else (m["destination"] if m["phase"]=="customer" else m["base"])
        m["route"]=astar(m["position"],target); m["route_index"]=0; m["dynamic_reroutes"]=m.get("dynamic_reroutes",0)+1
        m["route_reason"]="Manual dynamic A* reroute"; m["message"]="🧭 Route recalculated successfully."
        push_alert("info","Route recalculated",m["message"],mission_id)
        return jsonify({"success":True,"mission":mission_dict(m)})

@app.route("/api/alerts")
def api_alerts():
    with lock: return jsonify({"alerts":system_alerts[:25]})

@app.route("/api/health")
def api_health():
    with lock:
        return jsonify({"drones":[{"id":d["id"],"health":d.get("health",95),"battery":d["battery"],"temperature":d.get("temperature",32),"gps_signal":d.get("gps_signal",95),"connection":d.get("connection",95),"flight_hours":d.get("flight_hours",40),"maintenance":"SERVICE SOON" if d.get("health",95)<70 else "NORMAL"} for d in DRONES.values()]})

@app.route("/api/radar/<mission_id>")
def api_radar(mission_id):
    with lock:
        m=missions.get(mission_id)
        if not m: return jsonify({"success":False,"message":"Mission not found."}),404
        o=m.get("obstacle_event")
        return jsonify({"success":True,"mission_id":mission_id,"drone_id":m["drone_id"],"obstacle":o,"events":m.get("obstacle_events",0),"status":m["status"],"position":m["position"],"heading":(m.get("attitude") or {}).get("desired_heading"),"radar_range_m":120,"radar_sweep_deg":(time.time()*110)%360,"weather_zone":m.get("weather_zone"),"weather_alert":m.get("weather_alert")})

@app.route("/api/customer-track/<mission_id>")
def api_customer_track(mission_id):
    with lock:
        m=missions.get(mission_id)
        if not m: return jsonify({"success":False,"message":"Mission not found."}),404
        return jsonify({"success":True,"mission_id":m["id"],"drone_id":m["drone_id"],"status":m["status"],"position":m["position"],"destination":m["destination"],"eta_minutes":m.get("eta_minutes",0),"battery":m["battery"],"altitude":m.get("flight_altitude",65),"verification_status":m.get("verification_status")})

@app.route("/api/tracking/all")
def api_all():
    with lock:
        return jsonify({
            "missions": [mission_dict(m) for m in missions.values()],
            "drones": list(DRONES.values()),
            "weather": weather_state,
        })


@app.route("/api/history")
def api_history():
    conn = db_connect()
    rows = conn.execute("""
        SELECT * FROM deliveries
        ORDER BY completed_at DESC
        LIMIT 200
    """).fetchall()
    conn.close()
    return jsonify({"history": [dict(r) for r in rows]})


@app.route("/api/stats")
def api_stats():
    conn = db_connect()
    total = conn.execute("SELECT COUNT(*) c FROM deliveries").fetchone()["c"]
    delivered = conn.execute("SELECT COUNT(*) c FROM deliveries WHERE status='DELIVERED'").fetchone()["c"]
    unavailable = conn.execute("SELECT COUNT(*) c FROM deliveries WHERE status='RECIPIENT_UNAVAILABLE'").fetchone()["c"]
    distance_total = conn.execute("SELECT COALESCE(SUM(total_distance),0) d FROM deliveries").fetchone()["d"]
    by_type = {}
    for row in conn.execute("SELECT parcel_type, COUNT(*) c FROM deliveries GROUP BY parcel_type"):
        by_type[row["parcel_type"]] = row["c"]
    by_drone = {}
    for row in conn.execute("SELECT drone_id, COUNT(*) c FROM deliveries GROUP BY drone_id"):
        by_drone[row["drone_id"]] = row["c"]
    conn.close()

    live_busy = sum(1 for d in DRONES.values() if d["busy"])

    return jsonify({
        "total_completed": total,
        "delivered": delivered,
        "recipient_unavailable": unavailable,
        "total_distance_km": round(distance_total, 2),
        "by_type": by_type,
        "by_drone": by_drone,
        "fleet_size": len(DRONES),
        "busy_now": live_busy,
        "available_now": len(DRONES) - live_busy,
        "customer_points": len(CUSTOMERS),
    })


@app.route("/api/reset", methods=["POST"])
def api_reset():
    with lock:
        missions.clear()
        for d in DRONES.values():
            d["busy"] = False
            d["mission_id"] = None
    return jsonify({"success": True})


@app.route("/api/history/clear", methods=["POST"])
def api_history_clear():
    conn = db_connect()
    conn.execute("DELETE FROM deliveries")
    conn.commit()
    conn.close()
    return jsonify({"success": True})


# ============================================================
# FRONTEND
# ============================================================

HTML = r"""
<!doctype html>
<html>
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>SkyDeliver Ultimate</title>
<link rel="stylesheet" href="https://unpkg.com/leaflet@1.9.4/dist/leaflet.css">
<style>
:root{--bg:#08111f;--panel:#101d30;--panel2:#15253b;--text:#edf4ff;--muted:#9db0c9;--accent:#41b3ff;--good:#28d17c;--warn:#ffbf3f;--danger:#ff5f6d}
*{box-sizing:border-box}
body{margin:0;background:#edf2f7;color:#182536;font-family:Inter,Arial,sans-serif}
header{height:72px;background:linear-gradient(120deg,#07111f,#12335a);color:#fff;display:flex;align-items:center;justify-content:space-between;padding:0 20px;box-shadow:0 3px 18px #0003}
.brand{font-size:22px;font-weight:800}.sub{font-size:12px;color:#b8cbe0;margin-top:3px}
.live{padding:8px 12px;border-radius:20px;background:#163b59;color:#9edaff;font-size:12px}
.layout{display:grid;grid-template-columns:330px 1fr;height:calc(100vh - 72px)}
.sidebar{background:#fff;border-right:1px solid #d8e0e9;padding:16px;overflow:auto}
h3{margin:0 0 10px;font-size:15px}
label{font-size:11px;font-weight:800;display:block;margin:9px 0 5px;text-transform:uppercase;color:#53657b}
select,button{width:100%;border:1px solid #cfd9e5;border-radius:8px;padding:10px;background:#fff}
button{cursor:pointer;font-weight:800;margin-top:7px}.primary{background:#087fca;color:#fff;border:0}.danger{background:#d94352;color:#fff;border:0}.dark{background:#15263b;color:#fff;border:0}
.section{padding-bottom:16px;margin-bottom:16px;border-bottom:1px solid #e5eaf0}
.kpis{display:grid;grid-template-columns:1fr 1fr;gap:8px}.kpi{background:#f5f8fb;border:1px solid #e2e8ef;padding:9px;border-radius:9px}.kpi b{display:block;font-size:19px}.kpi span{font-size:10px;color:#66778d}
#map{height:100%;width:100%}.mapwrap{position:relative}
.weather{position:absolute;top:12px;left:12px;z-index:1000;background:#fff;border-radius:13px;padding:12px 14px;box-shadow:0 5px 25px #0003;min-width:270px}
.weather .big{font-weight:900;font-size:17px}.weather .small{font-size:11px;color:#607086;margin-top:4px;line-height:1.6}
.dashboard{position:absolute;right:12px;top:12px;z-index:1000;width:390px;max-height:calc(100% - 24px);overflow:auto}
.card{background:#fff;border-radius:13px;padding:12px;margin-bottom:10px;box-shadow:0 4px 18px #0002}.title{display:flex;justify-content:space-between;font-weight:900}.pill{display:inline-block;padding:5px 8px;border-radius:20px;background:#edf6ff;color:#1268a0;font-size:10px;font-weight:900;margin-top:6px}.line{font-size:11px;color:#5b6d83;line-height:1.65}.bar{height:7px;background:#e6ebf1;border-radius:8px;overflow:hidden;margin:6px 0}.bar>i{display:block;height:100%;background:#16a36a}.att{background:#f7fafc;border-radius:8px;padding:8px;margin-top:7px;font-size:10px;line-height:1.65}.tabs{display:flex;gap:5px;margin-bottom:8px}.tabs button{font-size:11px;padding:8px}.verifyBox{background:#f0f8ff;border:1px solid #b9dfff;border-radius:10px;padding:10px;margin-top:10px;font-size:11px;line-height:1.6}.verifyBox img{width:150px;height:150px;display:block;margin:8px auto}.verifyRow{display:flex;gap:6px}.verifyRow input{flex:1;border:1px solid #cfd9e5;border-radius:8px;padding:9px}.addressBox{background:#f7fbff;border:1px solid #cfe5f5;border-radius:10px;padding:10px;margin-top:8px}.addressResults{max-height:180px;overflow:auto;margin-top:7px}.addressResult{padding:8px;border-bottom:1px solid #e6edf4;font-size:11px;cursor:pointer}.addressResult:hover{background:#eef8ff}.selectedAddress{font-size:10px;color:#1268a0;margin-top:7px;line-height:1.45}.history{font-size:10px;max-height:250px;overflow:auto}.historyrow{padding:8px 0;border-bottom:1px solid #edf0f4}.mono{font-family:ui-monospace,monospace}
@media(max-width:1000px){.layout{grid-template-columns:1fr;height:auto}.sidebar{height:auto}.mapwrap{height:720px}.dashboard{width:330px}.weather{min-width:230px}}
.radarPanel{position:absolute;left:12px;bottom:12px;z-index:1000;width:245px;background:#071525ee;color:#bff4ff;border:1px solid #2a9fd6;border-radius:14px;padding:10px;box-shadow:0 5px 25px #0005}.radarTitle{font-weight:900;font-size:12px}.radarScope{height:150px;width:150px;border:2px solid #25d99b;border-radius:50%;margin:8px auto;position:relative;overflow:hidden;background:radial-gradient(circle,#123a2d 0,#071c18 55%,#03100d 100%)}.radarCross{position:absolute;background:#25d99b55}.radarCross.h{height:1px;left:0;right:0;top:50%}.radarCross.v{width:1px;top:0;bottom:0;left:50%}.radarSweep{position:absolute;width:50%;height:2px;top:50%;left:50%;transform-origin:left center;background:#25d99b;animation:sweep 1.25s linear infinite}@keyframes sweep{to{transform:rotate(360deg)}}.radarDot{position:absolute;width:10px;height:10px;border-radius:50%;background:#ff4b5c;display:none;box-shadow:0 0 10px #ff4b5c}.historySlide{display:none;position:absolute;inset:0;background:#fff;z-index:2000;padding:22px;overflow:auto}.historyHead{display:flex;justify-content:space-between;font-size:18px;margin-bottom:15px}.historyHead button{width:auto}.historyTable{width:100%;border-collapse:collapse;font-size:12px}.historyTable th,.historyTable td{padding:9px;border-bottom:1px solid #e5eaf0;text-align:left}.speedBadge{background:#eef8ff;border-radius:8px;padding:6px;margin-top:6px;font-size:10px}.radarAlert{color:#ff6977;font-weight:900}</style>
</head>
<body>
<header>
<div><div class="brand">🚁 SkyDeliver Pro</div><div class="sub">Autonomous multi-drone last-mile delivery • live weather • A* • wind compensation • history</div></div>
<div class="live">● LIVE FLEET MONITORING</div>
</header>

<div class="layout">
<aside class="sidebar">
<div class="section">
<h3>📦 Create Delivery</h3>
<label>Parcel Type</label>
<select id="type">
<option value="food">🍔 Food</option>
<option value="medical">🏥 Medical</option>
<option value="gadget">📱 Gadget</option>
</select>
<label>Pickup Point</label><select id="pickup"></select>
<label>Customer Address (Hyderabad)</label>
<div class="addressBox">
<input id="addressSearch" type="text" placeholder="e.g. Road No 36, Jubilee Hills, Hyderabad" style="width:100%;border:1px solid #cfd9e5;border-radius:8px;padding:10px">
<button class="primary" onclick="searchAddress()">🔍 Find Address</button>
<div id="addressResults" class="addressResults"></div>
<div id="selectedAddress" class="selectedAddress">No address selected. Search and select a result.</div>
</div>
<label>Customer Zone / Point (fallback)</label><select id="customer"></select>
<label>Customer Phone Number</label>
<input id="customerPhone" type="tel" placeholder="e.g. 9876543210" style="width:100%;border:1px solid #cfd9e5;border-radius:8px;padding:10px">
<label>Order Payload / Package Weight (kg)</label>
<input id="payloadKg" type="number" min="0.1" max="10" step="0.1" value="1.5" placeholder="e.g. 1.5" style="width:100%;border:1px solid #cfd9e5;border-radius:8px;padding:10px">
<div class="verifyBox">⚖️ <b>Autonomous Payload Balancing:</b> SkyDeliver checks package weight against drone maximum payload before dispatch. The system selects a suitable drone and shows payload load %, remaining capacity and balance status.</div>
<button class="primary" onclick="createDelivery()">🚁 Dispatch Drone</button>
<button class="dark" onclick="dispatchDemoFleet()">⚡ Dispatch 3-Dron Demo</button>
<div id="verificationBox" class="verifyBox">🔐 Customer OTP/QR will appear here when the drone reaches the customer.</div>
<button class="dark" onclick="showAlerts()">🚨 System Alerts</button>
<button class="dark" onclick="showHealth()">🔧 Drone Health</button>
<button class="dark" onclick="openHistorySlide()">📜 Open History Slide</button>
<button class="danger" onclick="resetLive()">Reset Live Missions</button>
</div>

<div class="section">
<h3>🌦️ Automatic Weather</h3>
<div id="weatherSide">Loading live weather...</div>
</div>

<div class="section">
<h3>📊 Fleet Statistics</h3>
<div class="kpis">
<div class="kpi"><b id="total">0</b><span>COMPLETED</span></div>
<div class="kpi"><b id="busy">0</b><span>BUSY NOW</span></div>
<div class="kpi"><b id="points">0</b><span>DELIVERY POINTS</span></div>
<div class="kpi"><b id="km">0</b><span>TOTAL KM</span></div>
</div>
</div>

<div class="section">
<h3>🧭 Mission Model</h3>
<div class="line">
Base → Pickup → Package Pickup → Customer Point → Delivery → Base → Charging
<br><br>
A* creates the route. The live weather engine changes speed, battery usage and simulated attitude compensation.
</div>
</div>

<div class="section">
<h3>📜 History</h3>
<button onclick="loadHistory()">View Delivery History</button>
<button onclick="clearHistory()">Clear History</button>
<div id="history" class="history"></div>
</div>
</aside>

<main class="mapwrap">
<div id="map"></div>

<div class="weather">
<div class="big" id="weatherBig">🌐 Weather loading...</div>
<div class="small" id="weatherInfo">Connecting to live weather service...</div>
</div>

<div id="radarPanel" class="radarPanel"><div class="radarTitle">📡 AIRBORNE RADAR</div><div id="radarInfo">Select an active drone</div><div class="radarScope"><div class="radarCross h"></div><div class="radarCross v"></div><div class="radarSweep"></div><div id="radarDot" class="radarDot"></div></div></div>
<div id="historySlide" class="historySlide"><div class="historyHead"><b>📜 DELIVERY HISTORY</b><button onclick="closeHistorySlide()">✕</button></div><div id="historyFull">Loading...</div></div>

<div class="dashboard">
<div class="tabs">
<button onclick="showLive()">🚁 LIVE</button>
<button onclick="loadHistory()">📜 HISTORY</button>
</div>
<div id="missions"></div>
</div>
</main>
</div>

<script src="https://unpkg.com/leaflet@1.9.4/dist/leaflet.js"></script>
<script>
let map=L.map('map').setView([17.43,78.43],12);
L.tileLayer('https://server.arcgisonline.com/ArcGIS/rest/services/World_Imagery/MapServer/tile/{z}/{y}/{x}',{maxZoom:19,attribution:'Tiles © Esri — Source: Esri, Maxar, Earthstar Geographics, and the GIS User Community'}).addTo(map);
let data=null, markers={}, routes={}, customerLayer=[];
let selectedAddress=null, selectedAddressMarker=null;
const colors={food:'#ef476f',medical:'#118ab2',gadget:'#8338ec'};

function markerIcon(icon){
return L.divIcon({className:'droneIcon',html:`<div style="font-size:27px;text-shadow:0 2px 5px #0008">${icon}</div>`,iconSize:[30,30],iconAnchor:[15,15]});
}

async function loadData(){
const r=await fetch('/api/data');data=await r.json();
populatePickups();populateCustomers();drawStatic();
document.getElementById('points').textContent=data.customer_count;
}
function populatePickups(){
const type=document.getElementById('type').value, s=document.getElementById('pickup');s.innerHTML='';
(data.pickups[type]||[]).forEach(p=>{let o=document.createElement('option');o.value=p.id;o.textContent=p.name+' • '+p.item;s.appendChild(o)});
}
function populateCustomers(){
const s=document.getElementById('customer');s.innerHTML='';
data.customers.forEach(c=>{let o=document.createElement('option');o.value=c.id;o.textContent=c.zone+' • '+c.name;s.appendChild(o)});
}
document.getElementById('type').onchange=populatePickups;

function drawStatic(){
data.bases.forEach(b=>L.marker([b.lat,b.lon]).addTo(map).bindPopup('🏠 <b>'+b.name+'</b><br>Drone base'));
data.chargers.forEach(c=>L.circleMarker([c.lat,c.lon],{radius:7}).addTo(map).bindPopup('⚡ '+c.name));
(data.no_fly_zones||[]).forEach(z=>L.circle([z.lat,z.lon],{radius:z.radius_km*1000,color:'#d90429',fillColor:'#d90429',fillOpacity:.12}).addTo(map).bindPopup('🚫 '+z.name));
Object.values(data.pickups).flat().forEach(p=>L.circleMarker([p.lat,p.lon],{radius:5}).addTo(map).bindPopup('📦 <b>'+p.name+'</b><br>'+p.item));
data.customers.forEach(c=>{
let m=L.circleMarker([c.lat,c.lon],{radius:3,weight:1,fillOpacity:.45}).addTo(map);
m.bindPopup('👤 <b>'+c.name+'</b><br>'+c.id);customerLayer.push(m);
});
}

async function searchAddress(){
const q=document.getElementById('addressSearch').value.trim();
if(q.length<4){alert('Please enter a fuller Hyderabad address.');return;}
const box=document.getElementById('addressResults'); box.innerHTML='<div class="addressResult">🔎 Searching Hyderabad...</div>';
try{
 const r=await fetch('/api/geocode?q='+encodeURIComponent(q)); const x=await r.json();
 if(!x.success){box.innerHTML='<div class="addressResult">❌ '+(x.message||'Address not found')+'</div>';return;}
 box.innerHTML=x.results.map((v,i)=>`<div class="addressResult" onclick='selectAddress(${JSON.stringify(v)})'>📍 <b>Result ${i+1}</b><br>${v.display_name}</div>`).join('');
}catch(e){box.innerHTML='<div class="addressResult">❌ Address search failed. Check internet.</div>';}
}
function selectAddress(v){
 selectedAddress=v;
 document.getElementById('selectedAddress').innerHTML='📍 <b>Selected:</b> '+v.display_name+'<br>Lat: '+Number(v.lat).toFixed(6)+' • Lon: '+Number(v.lon).toFixed(6);
 document.getElementById('addressResults').innerHTML='';
 if(selectedAddressMarker) map.removeLayer(selectedAddressMarker);
 selectedAddressMarker=L.marker([v.lat,v.lon]).addTo(map).bindPopup('<b>📍 Customer Delivery Address</b><br>'+v.display_name).openPopup();
 map.flyTo([v.lat,v.lon],17,{duration:1.5});
}

async function createDelivery(){
if(!selectedAddress){
 const useFallback=confirm('No address selected. Use the selected fallback customer point instead?');
 if(!useFallback)return;
}
const body={parcel_type:document.getElementById('type').value,pickup_id:document.getElementById('pickup').value,customer_id:document.getElementById('customer').value,recipient_available:'true',customer_phone:document.getElementById('customerPhone').value,payload_kg:parseFloat(document.getElementById('payloadKg').value||'1')};
if(selectedAddress){body.customer_lat=selectedAddress.lat;body.customer_lon=selectedAddress.lon;body.customer_name=selectedAddress.display_name;body.customer_address=selectedAddress.display_name;}
const r=await fetch('/api/start',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify(body)});
const x=await r.json();
if(!x.success) alert(x.message);
else {
 document.getElementById('verificationBox').innerHTML=`<b>🚁 DRONE DISPATCHED</b><br>
 Drone: <b>${x.mission.drone_id}</b><br>
 Mission: ${x.mission.id}<br>
 Parcel mass: <b>${Number(x.mission.payload_kg).toFixed(1)} kg</b> / ${Number(x.mission.drone_max_payload_kg).toFixed(1)} kg<br>
 Route: Base → Pickup → Customer → Return<br>
 Weather: ☀️ Sunny / Clear<br>
 <b>Live tracking started. OTP/QR will appear automatically when the drone reaches the customer.</b>`;
 if(x.mission.destination.lat&&x.mission.destination.lon) map.flyTo([x.mission.destination.lat,x.mission.destination.lon],16,{duration:1.2});
}
}

async function showCustomerVerification(missionId){
const r=await fetch('/api/customer/'+missionId); const x=await r.json();
if(!x.success){alert(x.message);return;}
document.getElementById('verificationBox').innerHTML=`<b>🔐 CUSTOMER VERIFICATION READY</b><br>Mission: ${x.mission_id}<br>Phone: ${x.phone||'not supplied'}<br>Status: ${x.status}<br><br><b>Demo OTP: ${x.otp}</b><br><b>⏱️ Valid for 60 seconds</b><br><img src="${x.qr_image||''}"><div class="verifyRow"><input id="verifyOtp" placeholder="Enter customer OTP"><button onclick="verifyCustomer('${x.mission_id}')">Verify</button></div>`;
}

async function verifyCustomer(missionId){
const otp=document.getElementById('verifyOtp').value.trim();
const r=await fetch('/api/verify',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify({mission_id:missionId,otp})});
const x=await r.json();
alert(x.message);
if(x.success){document.getElementById('verificationBox').innerHTML='<b>✅ CUSTOMER VERIFIED</b><br>OTP accepted. Package handover authorized.';}
}

async function dispatchDemoFleet(){
const types=['food','medical','gadget'];
for(let i=0;i<types.length;i++){
let p=data.pickups[types[i]][0], c=data.customers[i*20];
await fetch('/api/start',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify({parcel_type:types[i],pickup_id:p.id,customer_id:c.id,recipient_available:true,customer_phone:""})});
}
alert('Three different delivery missions dispatched simultaneously.');
}

function openHistorySlide(){document.getElementById('historySlide').style.display='block';loadHistoryFull();}
function closeHistorySlide(){document.getElementById('historySlide').style.display='none';}
async function loadHistoryFull(){const r=await fetch('/api/history');const x=await r.json();const h=document.getElementById('historyFull');if(!x.history.length){h.innerHTML='<p>No completed deliveries yet.</p>';return;}h.innerHTML='<table class="historyTable"><tr><th>Mission</th><th>Drone</th><th>Type</th><th>Route</th><th>KM</th><th>Avg Speed</th><th>Weather Changes</th><th>Obstacles</th><th>Status</th><th>Completed</th></tr>'+x.history.map(v=>`<tr><td>${v.id}</td><td>${v.drone_id}</td><td>${v.parcel_type}</td><td>${v.pickup_name} → ${v.customer_name}</td><td>${Number(v.total_distance||0).toFixed(2)}</td><td>${Number(v.avg_speed||0).toFixed(1)} km/h</td><td>${v.weather_changes||0}</td><td>${v.obstacle_events||0}</td><td>${v.status}</td><td>${v.completed_at}</td></tr>`).join('')+'</table>';}
function updateRadar(m){const panel=document.getElementById('radarPanel'),info=document.getElementById('radarInfo'),dot=document.getElementById('radarDot');if(!m){info.textContent='Select an active drone';dot.style.display='none';return;}const o=m.obstacle_event;info.innerHTML=`<b>${m.drone_id}</b><br>📡 360° MOVING RADAR • Range 120 m<br>${o?`<span class="radarAlert">⚠ ${o.type} • ${Math.round(o.distance_m)} m • MOVING</span>`:'🟢 Airspace scan clear'}<br>Events: ${m.obstacle_events||0} • ${m.status}<br>🌦️ ${m.weather_zone||'Normal corridor'}`;if(o){dot.style.display='block';const ang=((o.bearing||0)+(Date.now()/45))%360*Math.PI/180;const r=Math.min(58,Math.max(10,(o.distance_m||50)/120*58));dot.style.left=(70+Math.sin(ang)*r)+'px';dot.style.top=(70-Math.cos(ang)*r)+'px';}else dot.style.display='none';}

async function showAlerts(){const r=await fetch('/api/alerts');const x=await r.json();alert((x.alerts||[]).map(a=>a.level.toUpperCase()+" | "+a.title+"\n"+a.message).join("\n\n")||"No alerts.");}
async function showHealth(){const r=await fetch('/api/health');const x=await r.json();alert((x.drones||[]).map(d=>d.id+": health "+Math.round(d.health)+"% | battery "+Math.round(d.battery)+"% | temp "+Math.round(d.temperature)+"°C | GPS "+Math.round(d.gps_signal)+"% | link "+Math.round(d.connection)+"% | "+d.maintenance).join("\n"));}
async function manualReroute(id){const r=await fetch('/api/reroute/'+id,{method:'POST'});const x=await r.json();if(!x.success)alert(x.message);}
async function emergencyReturn(id){const reason=prompt('Emergency reason:','Operator safety command');if(reason===null)return;const r=await fetch('/api/emergency/'+id,{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify({reason})});const x=await r.json();alert(x.message||'Emergency command sent.');}

function weatherHtml(w){
if(!w)return '';
return `<b>${w.emoji} ${w.condition}</b><br>Temperature: ${w.temperature??'-'} °C<br>Wind: ${w.wind_speed??0} km/h • Gust ${w.wind_gusts??0} km/h<br>Direction: ${w.wind_direction??0}° • Rain ${w.rain??0} mm<br>Updated: ${w.updated_at??'-'}`;
}
async function updateWeather(){
try{
let r=await fetch('/api/weather'),w=await r.json();
document.getElementById('weatherBig').textContent=(w.emoji||'🌐')+' '+w.condition;
document.getElementById('weatherInfo').innerHTML=weatherHtml(w);
document.getElementById('weatherSide').innerHTML=weatherHtml(w);
}catch(e){}
}

let activeVerificationMission=null;

function renderVerification(x){
if(!x || !x.success)return;
const remaining=Math.max(0,Number(x.otp_remaining_seconds||0));
const box=document.getElementById('verificationBox');
box.innerHTML=`<b>🔐 CUSTOMER VERIFICATION READY</b><br>
Mission: ${x.mission_id}<br>
Customer: ${x.customer?.name||'Customer'}<br>
Status: ${x.status}<br><br>
<b>Demo OTP: ${x.otp}</b><br>
<b>⏱️ Valid for <span id="otpCountdown">${remaining}</span> seconds</b><br>
<img src="${x.qr_image||''}" alt="Customer QR code">
<div class="verifyRow"><input id="verifyOtp" type="text" inputmode="numeric" autocomplete="one-time-code" placeholder="Enter 6-digit customer OTP" maxlength="6"><button onclick="verifyCustomer('${x.mission_id}')">Verify</button></div>
<div style="margin-top:6px;color:#607086">Scan the QR or enter the OTP before the timer reaches 0.</div>`;
const input=document.getElementById('verifyOtp');
if(input){input.focus(); input.addEventListener('input',()=>{input.value=input.value.replace(/\D/g,'').slice(0,6);});}
}

async function refreshVerification(list){
const waiting=list.find(m=>m.status==='WAITING_FOR_OTP' || m.status==='ARRIVED_AT_DESTINATION');
if(!waiting){
activeVerificationMission=null;
const box=document.getElementById('verificationBox');
if(box && !box.querySelector('.dispatchNotice')) box.innerHTML='<span class="dispatchNotice">🚁 Waiting for a drone to reach a customer. OTP/QR verification will appear automatically.</span>';
return;
}
try{
const r=await fetch('/api/customer/'+waiting.id); const x=await r.json();
if(x.success){
  if(activeVerificationMission!==waiting.id){
    activeVerificationMission=waiting.id;
    renderVerification(x);
  } else {
    const cd=document.getElementById('otpCountdown');
    if(cd) cd.textContent=Math.max(0,Number(x.otp_remaining_seconds||0));
    const statusLine=document.getElementById('verificationBox');
    const input=document.getElementById('verifyOtp');
    if(input && document.activeElement!==input && x.otp_remaining_seconds<=0) input.disabled=true;
  }
}
}catch(e){}
}

function updateMissions(list){
const root=document.getElementById('missions');
if(!list.length){root.innerHTML='<div class="card">No active missions. Dispatch a drone.</div>';refreshVerification(list);return}
root.innerHTML='';
list.forEach(m=>{
let a=m.attitude||{}, batt=Math.round(m.battery);
let card=document.createElement('div');card.className='card';
card.innerHTML=`
<div class="title"><span>${m.drone_id} ${m.parcel_type==='food'?'🍔':m.parcel_type==='medical'?'🏥':'📱'}</span><span>${batt}%</span></div>
<div class="pill">${m.status}</div>
<div class="line">
📦 ${m.item}<br>
⚖️ Parcel mass: <b>${Number(m.payload_kg||0).toFixed(1)} kg</b> / ${Number(m.drone_max_payload_kg||0).toFixed(1)} kg capacity<br>
📍 Pickup: ${m.pickup.name}<br>
👤 Destination: ${m.destination.name}<br>
🚀 Speed: ${m.speed} km/h<br>
⚙️ ${m.speed_reason||'Dynamic speed control'}<br>
📏 Mission: ${m.total_distance} km<br>
⏱️ ETA: ${m.eta_minutes} min<br>
🌦️ ${m.mission_weather?.condition||m.weather_at_creation.condition}<br>📍 Weather zone: ${m.weather_zone||'Normal'}<br>🚨 ${m.weather_alert||''}
</div>
<div class="bar"><i style="width:${batt}%"></i></div>
<div class="line">${m.message}</div>${m.obstacle_event?`<div class="att"><b class="radarAlert">📡 RADAR ALERT: ${m.obstacle_event.type}</b><br>Distance ${Math.round(m.obstacle_event.distance_m)} m • Drone moving aside • Bearing ${Math.round(m.obstacle_event.bearing)}°</div>`:''}<div class="att"><b>🛰️ LIVE AVIONICS</b><br>GPS ${Math.round(m.gps_signal||0)}% • ±${m.gps_accuracy_m||'-'} m • Link ${Math.round(m.connection||0)}%<br>Latency ${m.connection_latency_ms||'-'} ms • Altitude ${m.flight_altitude||'-'} m • Airspace ${m.collision_status||'CLEAR'}<br>A* reroutes ${m.dynamic_reroutes||0} • ${m.route_reason||'Initial route'}</div>
${m.status==='WAITING_FOR_OTP'?`<button class="primary" onclick="showCustomerVerification('${m.id}')">🔐 Customer OTP / QR • 60s</button>`:''}${m.status==='RECIPIENT_UNAVAILABLE'?`<div class="verifyBox">⏱️ <b>Customer unavailable.</b><br>60-second OTP/QR window expired. Package is returning to the parcel office.</div>`:''}${!['CHARGING','AVAILABLE'].includes(m.status)?`<button onclick="manualReroute('${m.id}')">🧭 Recalculate A*</button><button class="danger" onclick="emergencyReturn('${m.id}')">🚨 Emergency Return</button>`:''}
${a.wind_speed!==undefined?`
<div class="att">
<b>💨 WIND COMPENSATION</b><br>
Wind ${a.wind_speed} km/h • Gust ${a.wind_gusts} km/h<br>
Wind direction ${a.wind_direction}°<br>
Desired heading ${a.desired_heading}°<br>
Compensated heading <b>${a.compensated_heading}°</b><br>
Crosswind ${a.crosswind} km/h<br>
Correction ${a.correction_angle}°<br>
↔ Roll ${a.roll_angle}° • ↕ Pitch ${a.pitch_angle}° • Yaw ${a.yaw_angle}°<br>
<b>${a.message}</b>
</div>`:''}`;
root.appendChild(card);

if(!markers[m.id]){
markers[m.id]=L.marker([m.position.lat,m.position.lon],{icon:markerIcon(m.parcel_type==='food'?'🚁':m.parcel_type==='medical'?'🚁':'🚁')}).addTo(map);
}else markers[m.id].setLatLng([m.position.lat,m.position.lon]);
markers[m.id].bindPopup(`<b>${m.drone_id}</b><br>${m.status}<br>Battery ${batt}%`);
if(!routes[m.id]){
routes[m.id]=L.polyline(m.route.map(p=>[p.lat,p.lon]),{weight:3,opacity:.55,color:colors[m.parcel_type]||'#1687c7'}).addTo(map);
}else{
routes[m.id].setLatLngs(m.route.map(p=>[p.lat,p.lon]));
}
});
refreshVerification(list);
}

async function tick(){
try{
let r=await fetch('/api/tracking/all'),x=await r.json();
updateMissions(x.missions);updateWeather();updateRadar(x.missions.find(m=>m.obstacle_event)||x.missions[0]);
let s=await fetch('/api/stats'),st=await s.json();
document.getElementById('total').textContent=st.total_completed;
document.getElementById('busy').textContent=st.busy_now;
document.getElementById('km').textContent=st.total_distance_km;
}catch(e){console.log(e)}
}
async function loadHistory(){
let r=await fetch('/api/history'),x=await r.json();
let h=document.getElementById('history');
if(!x.history.length){h.innerHTML='No completed deliveries yet.';return}
h.innerHTML=x.history.map(v=>`<div class="historyrow"><b>${v.id}</b> • ${v.drone_id}<br>${v.parcel_type} • ${v.item}<br>${v.pickup_name} → ${v.customer_name}<br>${Number(v.total_distance).toFixed(2)} km • ${v.status}<br>${v.completed_at}</div>`).join('');
}
async function clearHistory(){if(!confirm('Clear all saved delivery history?'))return;await fetch('/api/history/clear',{method:'POST'});loadHistory();}
async function resetLive(){await fetch('/api/reset',{method:'POST'});Object.values(markers).forEach(m=>map.removeLayer(m));Object.values(routes).forEach(r=>map.removeLayer(r));markers={};routes={};}
function showLive(){document.getElementById('missions').scrollTop=0}
loadData();updateWeather();tick();setInterval(tick,1000);setInterval(updateWeather,60000);
</script>
</body>
</html>
"""


if __name__ == "__main__":
    print("=" * 70)
    print("SKYDELIVER PRO - AUTONOMOUS MULTI-DRONE DELIVERY")
    print("=" * 70)
    print(f"Customer delivery points: {len(CUSTOMERS)}")
    print(f"Fleet size: {len(DRONES)}")
    print("Mission: BASE -> PICKUP -> CUSTOMER -> BASE -> CHARGE")
    print("Live weather: Open-Meteo")
    print("Web app: http://127.0.0.1:5000")
    print("=" * 70)
    app.run(host="0.0.0.0", port=5000, debug=True, use_reloader=False)
