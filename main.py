import streamlit as st
import folium
from streamlit_folium import folium_static, st_folium
from folium import plugins
import random
import time
import math
import json
import os
from datetime import datetime
import pandas as pd
import copy
import heapq

# ==================== 页面配置 ====================
st.set_page_config(page_title="无人机地面站系统 - 平行偏移绕行", layout="wide")

# ==================== 坐标 ====================
SCHOOL_CENTER_GCJ = [118.7490, 32.2340]
DEFAULT_A_GCJ = [118.746956, 32.232945]
DEFAULT_B_GCJ = [118.751589, 32.235204]

GAODE_SATELLITE_URL = "https://webst01.is.autonavi.com/appmaptile?style=6&x={x}&y={y}&z={z}"
GAODE_VECTOR_URL = "https://webrd02.is.autonavi.com/appmaptile?lang=zh_cn&size=1&scale=1&style=8&x={x}&y={y}&z={z}"

# ==================== 坐标系转换 ====================
def gcj02_to_wgs84(lng, lat):
    a = 6378245.0
    ee = 0.00669342162296594323
    if out_of_china(lng, lat):
        return lng, lat
    dlat = transform_lat(lng - 105.0, lat - 35.0)
    dlng = transform_lng(lng - 105.0, lat - 35.0)
    radlat = lat / 180.0 * math.pi
    magic = math.sin(radlat)
    magic = 1 - ee * magic * magic
    sqrtmagic = math.sqrt(magic)
    dlat = (dlat * 180.0) / ((a * (1 - ee)) / (magic * sqrtmagic) * math.pi)
    dlng = (dlng * 180.0) / (a / sqrtmagic * math.cos(radlat) * math.pi)
    mglat = lat + dlat
    mglng = lng + dlng
    return lng * 2 - mglng, lat * 2 - mglat

def wgs84_to_gcj02(lng, lat):
    a = 6378245.0
    ee = 0.00669342162296594323
    if out_of_china(lng, lat):
        return lng, lat
    dlat = transform_lat(lng - 105.0, lat - 35.0)
    dlng = transform_lng(lng - 105.0, lat - 35.0)
    radlat = lat / 180.0 * math.pi
    magic = math.sin(radlat)
    magic = 1 - ee * magic * magic
    sqrtmagic = math.sqrt(magic)
    dlat = (dlat * 180.0) / ((a * (1 - ee)) / (magic * sqrtmagic) * math.pi)
    dlng = (dlng * 180.0) / (a / sqrtmagic * math.cos(radlat) * math.pi)
    mglat = lat + dlat
    mglng = lng + dlng
    return mglng, mglat

def transform_lat(lng, lat):
    ret = -100.0 + 2.0 * lng + 3.0 * lat + 0.2 * lat * lat + 0.1 * lng * lat + 0.2 * math.sqrt(abs(lng))
    ret += (20.0 * math.sin(6.0 * lng * math.pi) + 20.0 * math.sin(2.0 * lng * math.pi)) * 2.0 / 3.0
    ret += (20.0 * math.sin(lat * math.pi) + 40.0 * math.sin(lat / 3.0 * math.pi)) * 2.0 / 3.0
    ret += (160.0 * math.sin(lat / 12.0 * math.pi) + 320 * math.sin(lat * math.pi / 30.0)) * 2.0 / 3.0
    return ret

def transform_lng(lng, lat):
    ret = 300.0 + lng + 2.0 * lat + 0.1 * lng * lng + 0.1 * lng * lat + 0.1 * math.sqrt(abs(lng))
    ret += (20.0 * math.sin(6.0 * lng * math.pi) + 20.0 * math.sin(2.0 * lng * math.pi)) * 2.0 / 3.0
    ret += (20.0 * math.sin(lng * math.pi) + 40.0 * math.sin(lng / 3.0 * math.pi)) * 2.0 / 3.0
    ret += (150.0 * math.sin(lng / 12.0 * math.pi) + 300.0 * math.sin(lng / 30.0 * math.pi)) * 2.0 / 3.0
    return ret

def out_of_china(lng, lat):
    return not (72.004 <= lng <= 137.8347 and 0.8293 <= lat <= 55.8271)

# ==================== 几何辅助函数 ====================
def point_in_polygon(point, polygon):
    x, y = point
    inside = False
    n = len(polygon)
    for i in range(n):
        x1, y1 = polygon[i]
        x2, y2 = polygon[(i + 1) % n]
        if ((y1 > y) != (y2 > y)) and (x < (x2 - x1) * (y - y1) / (y2 - y1) + x1):
            inside = not inside
    return inside

def segments_intersect(p1, p2, p3, p4):
    def ccw(A, B, C):
        return (C[1]-A[1]) * (B[0]-A[0]) > (B[1]-A[1]) * (C[0]-A[0])
    return (ccw(p1, p3, p4) != ccw(p2, p3, p4)) and (ccw(p1, p2, p3) != ccw(p1, p2, p4))

def line_intersects_polygon(p1, p2, polygon):
    if point_in_polygon(p1, polygon) or point_in_polygon(p2, polygon):
        return True
    n = len(polygon)
    for i in range(n):
        p3 = polygon[i]
        p4 = polygon[(i + 1) % n]
        if segments_intersect(p1, p2, p3, p4):
            if not (p1 == p3 or p1 == p4 or p2 == p3 or p2 == p4):
                return True
    return False

def distance(p1, p2):
    return math.sqrt((p1[0]-p2[0])**2 + (p1[1]-p2[1])**2)

# ==================== 障碍物高度与阻挡判断 ====================
def is_obstacle_blocking(obs, flight_height, safe_radius):
    obs_height = obs.get('height', 20)
    return flight_height <= obs_height + safe_radius

def is_path_blocked(p1, p2, obstacles_gcj, flight_height, safe_radius):
    for obs in obstacles_gcj:
        coords = obs.get('polygon', [])
        if coords and len(coords) >= 3:
            if is_obstacle_blocking(obs, flight_height, safe_radius):
                if line_intersects_polygon(p1, p2, coords):
                    return True
    return False

# ==================== 平行偏移绕行（已修复：左右完全分离） ====================
def generate_parallel_offset_path(start, end, obstacles_gcj, flight_height, safe_radius, side='left'):
    dx = end[0] - start[0]
    dy = end[1] - start[1]
    length = math.hypot(dx, dy)
    if length < 1e-8:
        return None

    ux = dx / length
    uy = dy / length

    if side == 'left':
        offset_x = -uy
        offset_y = ux
    else:
        offset_x = uy
        offset_y = -ux

    offset_m = safe_radius * 3.0
    offset_deg = offset_m / 111000.0

    for k in range(1, 15):
        offset = offset_deg * k
        p_off = [
            (start[0] + end[0])/2 + offset_x * offset,
            (start[1] + end[1])/2 + offset_y * offset
        ]
        path = [start.copy(), p_off, end.copy()]
        ok = True
        for i in range(len(path)-1):
            if is_path_blocked(path[i], path[i+1], obstacles_gcj, flight_height, safe_radius):
                ok = False
                break
        if ok:
            return path
    return None

# ==================== A* 路径规划（修复报错 + 安全半径生效） ====================
def astar_path(start, end, obstacles_gcj, flight_height, safe_radius):
    nodes = [start, end]
    safety = safe_radius / 111000.0 * 1.2  # 安全半径转为经纬度单位

    for obs in obstacles_gcj:
        if not is_obstacle_blocking(obs, flight_height, safe_radius):
            continue
        poly = obs.get('polygon', [])
        if len(poly) < 3:
            continue
        for i, (x, y) in enumerate(poly):
            # 前一个点
            prev_i = (i - 1) % len(poly)
            prev = poly[prev_i]
            # 下一个点
            next_i = (i + 1) % len(poly)
            next_p = poly[next_i]

            # 计算法线方向（向外）
            dx1 = -(y - prev[1])
            dy1 = x - prev[0]
            l1 = math.hypot(dx1, dy1)
            if l1 > 1e-8:
                dx1 /= l1
                dy1 /= l1
            nx1 = x + dx1 * safety
            ny1 = y + dy1 * safety

            dx2 = -(next_p[1] - y)
            dy2 = next_p[0] - x
            l2 = math.hypot(dx2, dy2)
            if l2 > 1e-8:
                dx2 /= l2
                dy2 /= l2
            nx2 = x + dx2 * safety
            ny2 = y + dy2 * safety

            nodes.append([nx1, ny1])
            nodes.append([nx2, ny2])

    # 去重节点
    unique_nodes = []
    for n in nodes:
        exists = False
        for u in unique_nodes:
            if abs(n[0] - u[0]) < 1e-6 and abs(n[1] - u[1]) < 1e-6:
                exists = True
                break
        if not exists:
            unique_nodes.append(n)

    # 构建邻接表
    graph = {i: [] for i in range(len(unique_nodes))}
    for i in range(len(unique_nodes)):
        for j in range(len(unique_nodes)):
            if i == j:
                continue
            if not is_path_blocked(unique_nodes[i], unique_nodes[j], obstacles_gcj, flight_height, safe_radius):
                graph[i].append((j, distance(unique_nodes[i], unique_nodes[j])))

    # 找到起点和终点索引
    start_i = -1
    end_i = -1
    for i, n in enumerate(unique_nodes):
        if abs(n[0] - start[0]) < 1e-6 and abs(n[1] - start[1]) < 1e-6:
            start_i = i
        if abs(n[0] - end[0]) < 1e-6 and abs(n[1] - end[1]) < 1e-6:
            end_i = i
    if start_i == -1 or end_i == -1:
        return [start, end]

    # A* 主循环
    open_heap = []
    heapq.heappush(open_heap, (0, start_i))
    came_from = {}
    g_score = {i: float('inf') for i in range(len(unique_nodes))}
    g_score[start_i] = 0
    f_score = {i: float('inf') for i in range(len(unique_nodes))}
    f_score[start_i] = distance(unique_nodes[start_i], unique_nodes[end_i])

    while open_heap:
        current_f, cur = heapq.heappop(open_heap)
        if cur == end_i:
            path = []
            while cur in came_from:
                path.append(unique_nodes[cur])
                cur = came_from[cur]
            path.append(unique_nodes[start_i])
            path.reverse()
            return path
        for neighbor, w in graph[cur]:
            new_g = g_score[cur] + w
            if new_g < g_score[neighbor]:
                came_from[neighbor] = cur
                g_score[neighbor] = new_g
                f_score[neighbor] = new_g + distance(unique_nodes[neighbor], unique_nodes[end_i])
                heapq.heappush(open_heap, (f_score[neighbor], neighbor))
    return [start, end]

def create_avoidance_path(start, end, obstacles_gcj, flight_height, safe_radius, strategy):
    if not is_path_blocked(start, end, obstacles_gcj, flight_height, safe_radius):
        return [start, end]

    if strategy == 'left':
        p = generate_parallel_offset_path(start, end, obstacles_gcj, flight_height, safe_radius, 'left')
        if p: return p
        p = generate_parallel_offset_path(start, end, obstacles_gcj, flight_height, safe_radius, 'right')
        if p: return p
        return astar_path(start, end, obstacles_gcj, flight_height, safe_radius)

    elif strategy == 'right':
        p = generate_parallel_offset_path(start, end, obstacles_gcj, flight_height, safe_radius, 'right')
        if p: return p
        p = generate_parallel_offset_path(start, end, obstacles_gcj, flight_height, safe_radius, 'left')
        if p: return p
        return astar_path(start, end, obstacles_gcj, flight_height, safe_radius)

    else:
        return astar_path(start, end, obstacles_gcj, flight_height, safe_radius)

# ==================== 障碍物管理 ====================
def save_obstacles_to_cache():
    if 'saved_obstacles' not in st.session_state:
        st.session_state.saved_obstacles = []
    st.session_state.saved_obstacles = copy.deepcopy(st.session_state.obstacles_gcj)
    st.success(f"已保存 {len(st.session_state.obstacles_gcj)} 个障碍物到缓存")

def load_obstacles_from_cache():
    if 'saved_obstacles' not in st.session_state or not st.session_state.saved_obstacles:
        st.warning("缓存中无障碍物，请先保存")
        return False
    st.session_state.obstacles_gcj = st.session_state.saved_obstacles
    st.success(f"已从缓存加载 {len(st.session_state.obstacles_gcj)} 个障碍物")
    return True

# ==================== 心跳包模拟器 ====================
class HeartbeatSimulator:
    def __init__(self, start_point_gcj):
        self.history = []
        self.current_pos = start_point_gcj.copy()
        self.path = [start_point_gcj.copy()]
        self.path_index = 0
        self.simulating = False
        self.flight_altitude = 50
        self.speed = 50
        self.progress = 0.0
        self.total_distance = 0.0
        self.distance_traveled = 0.0

    def set_path(self, path, altitude=50, speed=50):
        self.path = path
        self.path_index = 0
        self.current_pos = path[0].copy()
        self.flight_altitude = altitude
        self.speed = speed
        self.simulating = True
        self.progress = 0.0
        self.distance_traveled = 0.0
        self.total_distance = 0.0
        for i in range(len(path)-1):
            self.total_distance += distance(path[i], path[i+1])

    def update_and_generate(self):
        if self.simulating and self.path_index < len(self.path)-1:
            target = self.path[self.path_index+1]
            dx = target[0] - self.current_pos[0]
            dy = target[1] - self.current_pos[1]
            dist_to_target = math.hypot(dx, dy)
            step = 0.00015 + (self.speed/100)*0.0005
            if dist_to_target < step:
                self.distance_traveled += dist_to_target
                self.current_pos = target.copy()
                self.path_index += 1
            else:
                ratio = step / dist_to_target
                self.current_pos[0] += dx * ratio
                self.current_pos[1] += dy * ratio
                self.distance_traveled += step
            if self.total_distance > 0:
                self.progress = min(1.0, self.distance_traveled / self.total_distance)
            if self.path_index >= len(self.path)-1:
                self.simulating = False
                self.progress = 1.0
        else:
            self.simulating = False
            self.progress = 1.0
        altitude = self.flight_altitude + random.randint(-5,5) if self.simulating else random.randint(0,10)
        speed_display = round(self.speed * 0.1, 1) if self.simulating else 0
        data = {
            "timestamp": datetime.now().strftime("%H:%M:%S"),
            "lng": self.current_pos[0],
            "lat": self.current_pos[1],
            "altitude": altitude,
            "voltage": round(random.uniform(11.5,12.8),1),
            "satellites": random.randint(8,14),
            "speed": speed_display,
            "progress": self.progress,
            "distance_traveled": self.distance_traveled,
            "total_distance": self.total_distance,
            "simulating": self.simulating
        }
        self.history.insert(0, data)
        if len(self.history) > 200:
            self.history.pop()
        return data

# ==================== 创建地图 ====================
def create_planning_map(center_gcj, points_gcj, obstacles_gcj, flight_history=None, planned_path=None, map_type="satellite", straight_blocked=True):
    if map_type == "satellite":
        tiles = GAODE_SATELLITE_URL
        attr = "高德卫星地图"
    else:
        tiles = GAODE_VECTOR_URL
        attr = "高德矢量地图"
    m = folium.Map(location=[center_gcj[1], center_gcj[0]], zoom_start=16, tiles=tiles, attr=attr)
    draw = plugins.Draw(
        export=True, position='topleft',
        draw_options={'polygon': {'allowIntersection': False, 'showArea': True, 'color': '#ff0000', 'fillColor': '#ff0000', 'fillOpacity': 0.4},
                      'polyline': False, 'rectangle': False, 'circle': False, 'marker': False, 'circlemarker': False},
        edit_options={'edit': True, 'remove': True}
    )
    m.add_child(draw)
    for i, obs in enumerate(obstacles_gcj):
        coords = obs.get('polygon', [])
        if coords and len(coords) >= 3:
            popup_text = f"🚧 {obs.get('name', f'障碍物{i+1}')}\n高度: {obs.get('height', 20)}m"
            folium.Polygon([[c[1], c[0]] for c in coords], color="red", weight=3, fill=True, fill_color="red", fill_opacity=0.4, popup=popup_text).add_to(m)
    if points_gcj.get('A'):
        folium.Marker([points_gcj['A'][1], points_gcj['A'][0]], popup="🟢 起点", icon=folium.Icon(color="green", icon="play", prefix="fa")).add_to(m)
    if points_gcj.get('B'):
        folium.Marker([points_gcj['B'][1], points_gcj['B'][0]], popup="🔴 终点", icon=folium.Icon(color="red", icon="stop", prefix="fa")).add_to(m)
    if planned_path and len(planned_path) > 1:
        path_locations = [[p[1], p[0]] for p in planned_path]
        folium.PolyLine(path_locations, color="green", weight=5, opacity=0.9, popup="✈️ 智能避障航线").add_to(m)
        for i, point in enumerate(planned_path[1:-1]):
            folium.CircleMarker([point[1], point[0]], radius=4, color="green", fill=True, fill_color="white", fill_opacity=0.8, popup=f"航点 {i+1}").add_to(m)
    if points_gcj.get('A') and points_gcj.get('B'):
        if not straight_blocked:
            folium.PolyLine([[points_gcj['A'][1], points_gcj['A'][0]], [points_gcj['B'][1], points_gcj['B'][0]]], color="blue", weight=2, opacity=0.5, dash_array='5, 5', popup="直线航线").add_to(m)
        else:
            folium.PolyLine([[points_gcj['A'][1], points_gcj['A'][0]], [points_gcj['B'][1], points_gcj['B'][0]]], color="gray", weight=2, opacity=0.4, dash_array='5, 5', popup="⚠️ 直线被阻挡").add_to(m)
    if flight_history and len(flight_history) > 1:
        trail = [[p[1], p[0]] for p in flight_history if len(p) >= 2]
        if len(trail) > 1:
            folium.PolyLine(trail, color="orange", weight=2, opacity=0.6, popup="历史轨迹").add_to(m)
    return m

# ==================== 主程序 ====================
def main():
    st.title("🏫 无人机地面站系统 - 平行偏移绕行")
    st.markdown("---")

    if "points_gcj" not in st.session_state:
        st.session_state.points_gcj = {'A': DEFAULT_A_GCJ.copy(), 'B': DEFAULT_B_GCJ.copy()}
    if "obstacles_gcj" not in st.session_state:
        st.session_state.obstacles_gcj = []
    if "saved_obstacles" not in st.session_state:
        st.session_state.saved_obstacles = []
    if "heartbeat_sim" not in st.session_state:
        st.session_state.heartbeat_sim = HeartbeatSimulator(st.session_state.points_gcj['A'].copy())
    if "last_hb_time" not in st.session_state:
        st.session_state.last_hb_time = time.time()
    if "simulation_running" not in st.session_state:
        st.session_state.simulation_running = False
    if "flight_altitude" not in st.session_state:
        st.session_state.flight_altitude = 50
    if "flight_history" not in st.session_state:
        st.session_state.flight_history = []
    if "planned_path" not in st.session_state:
        st.session_state.planned_path = None
    if "pending_polygon" not in st.session_state:
        st.session_state.pending_polygon = None
    if "pending_height" not in st.session_state:
        st.session_state.pending_height = 20

    st.sidebar.title("🎛️ 导航菜单")
    page = st.sidebar.radio("选择功能模块", ["🗺️ 航线规划", "📡 飞行监控", "🚧 障碍物管理"])
    map_type_choice = st.sidebar.radio("🗺️ 地图类型", ["卫星影像", "矢量街道"], index=0)
    map_type = "satellite" if map_type_choice == "卫星影像" else "vector"

    st.sidebar.markdown("---")
    st.sidebar.subheader("⚙️ 无人机参数")
    drone_speed = st.sidebar.slider("飞行速度系数", min_value=10, max_value=100, value=50, step=5)
    safe_radius = st.sidebar.number_input("安全半径 (米)", min_value=1, max_value=30, value=5, step=1)
    flight_alt = st.sidebar.number_input("飞行高度 (米)", min_value=0, max_value=200, value=st.session_state.flight_altitude, step=5)
    st.session_state.flight_altitude = flight_alt

    st.sidebar.markdown("---")
    st.sidebar.subheader("🔄 绕行策略")
    strategy = st.sidebar.radio("选择避障方式", ["最佳航线 (A*)", "向左绕行", "向右绕行"], index=0)
    strategy_map = {"最佳航线 (A*)": "best", "向左绕行": "left", "向右绕行": "right"}
    selected_strategy = strategy_map[strategy]

    st.sidebar.markdown("---")
    obs_count = len(st.session_state.obstacles_gcj)
    straight_blocked = is_path_blocked(
        st.session_state.points_gcj['A'],
        st.session_state.points_gcj['B'],
        st.session_state.obstacles_gcj,
        st.session_state.flight_altitude,
        safe_radius
    )
    st.sidebar.info(f"🏫 校园区域\n🚧 障碍物: {obs_count}\n📌 直线: {'🚫 被阻挡' if straight_blocked else '✅ 畅通'}")

    if st.sidebar.button("🔄 刷新数据", use_container_width=True):
        st.session_state.planned_path = create_avoidance_path(
            st.session_state.points_gcj['A'],
            st.session_state.points_gcj['B'],
            st.session_state.obstacles_gcj,
            st.session_state.flight_altitude,
            safe_radius,
            selected_strategy
        )
        st.rerun()

    # ==================== 航线规划 ====================
    if page == "🗺️ 航线规划":
        st.header("🗺️ 航线规划 - 智能避障")
        if straight_blocked:
            st.warning(f"⚠️ 直线航线被建筑物阻挡！当前飞行高度 {flight_alt}m，某些障碍物高于此高度+安全半径。")
        else:
            st.success(f"✅ 直线航线畅通无阻 (飞行高度 {flight_alt}m)")

        col1, col2 = st.columns([1, 1.5])
        with col1:
            st.subheader("🎮 控制面板")
            st.markdown("#### 🟢 起点 A")
            a_lat = st.number_input("纬度", value=st.session_state.points_gcj['A'][1], format="%.6f", key="a_lat")
            a_lng = st.number_input("经度", value=st.session_state.points_gcj['A'][0], format="%.6f", key="a_lng")
            if st.button("📍 设置 A 点", use_container_width=True):
                st.session_state.points_gcj['A'] = [a_lng, a_lat]
                st.session_state.planned_path = create_avoidance_path(
                    st.session_state.points_gcj['A'], st.session_state.points_gcj['B'],
                    st.session_state.obstacles_gcj, flight_alt, safe_radius, selected_strategy
                )
                st.rerun()

            st.markdown("#### 🔴 终点 B")
            b_lat = st.number_input("纬度", value=st.session_state.points_gcj['B'][1], format="%.6f", key="b_lat")
            b_lng = st.number_input("经度", value=st.session_state.points_gcj['B'][0], format="%.6f", key="b_lng")
            if st.button("📍 设置 B 点", use_container_width=True):
                st.session_state.points_gcj['B'] = [b_lng, b_lat]
                st.session_state.planned_path = create_avoidance_path(
                    st.session_state.points_gcj['A'], st.session_state.points_gcj['B'],
                    st.session_state.obstacles_gcj, flight_alt, safe_radius, selected_strategy
                )
                st.rerun()

            st.markdown("#### 🏗️ 新障碍物高度")
            new_obs_height = st.number_input("高度 (米)", min_value=1, max_value=200, value=st.session_state.pending_height, step=5)
            st.session_state.pending_height = new_obs_height

            if st.button("➕ 添加障碍物（从当前圈选）", use_container_width=True):
                if st.session_state.pending_polygon and len(st.session_state.pending_polygon) >= 3:
                    st.session_state.obstacles_gcj.append({
                        "name": f"建筑物{len(st.session_state.obstacles_gcj)+1}",
                        "polygon": st.session_state.pending_polygon,
                        "height": st.session_state.pending_height
                    })
                    st.success(f"已添加障碍物（高度{st.session_state.pending_height}m）")
                    st.session_state.pending_polygon = None
                    st.session_state.planned_path = create_avoidance_path(
                        st.session_state.points_gcj['A'], st.session_state.points_gcj['B'],
                        st.session_state.obstacles_gcj, flight_alt, safe_radius, selected_strategy
                    )
                    st.rerun()
                else:
                    st.warning("请先在地图绘制多边形")

            if st.button("🔄 重新规划路径", use_container_width=True):
                st.session_state.planned_path = create_avoidance_path(
                    st.session_state.points_gcj['A'], st.session_state.points_gcj['B'],
                    st.session_state.obstacles_gcj, flight_alt, safe_radius, selected_strategy
                )
                st.rerun()

            st.markdown("#### ✈️ 飞行控制")
            c1, c2 = st.columns(2)
            with c1:
                if st.button("▶️ 开始飞行", use_container_width=True):
                    path = st.session_state.planned_path or [st.session_state.points_gcj['A'], st.session_state.points_gcj['B']]
                    st.session_state.heartbeat_sim.set_path(path, flight_alt, drone_speed)
                    st.session_state.simulation_running = True
                    st.session_state.flight_history = []
                    st.success("已开始飞行")
            with c2:
                if st.button("⏹️ 停止飞行", use_container_width=True):
                    st.session_state.simulation_running = False
                    st.session_state.heartbeat_sim.simulating = False

        with col2:
            st.subheader("🗺️ 规划地图")
            center = st.session_state.points_gcj['A'] or SCHOOL_CENTER_GCJ
            if st.session_state.planned_path is None:
                st.session_state.planned_path = create_avoidance_path(
                    st.session_state.points_gcj['A'], st.session_state.points_gcj['B'],
                    st.session_state.obstacles_gcj, flight_alt, safe_radius, selected_strategy
                )
            m = create_planning_map(center, st.session_state.points_gcj, st.session_state.obstacles_gcj,
                                   st.session_state.flight_history, st.session_state.planned_path, map_type, straight_blocked)
            output = st_folium(m, width=700, height=550, returned_objects=["last_active_drawing"])

            if output and output.get("last_active_drawing"):
                last = output["last_active_drawing"]
                if last and last.get("geometry") and last["geometry"]["type"] == "Polygon":
                    coords = last["geometry"]["coordinates"]
                    if coords:
                        poly = [[p[0], p[1]] for p in coords[0]]
                        if len(poly) >= 3:
                            st.session_state.pending_polygon = poly
                            st.success("已捕获多边形")

    # ==================== 飞行监控 ====================
    elif page == "📡 飞行监控":
        st.header("📡 飞行监控 - 实时心跳包")
        current_time = time.time()
        if st.session_state.simulation_running:
            if current_time - st.session_state.last_hb_time >= 0.2:
                st.session_state.heartbeat_sim.update_and_generate()
                st.session_state.last_hb_time = current_time
                st.rerun()

        if st.session_state.heartbeat_sim.history:
            latest = st.session_state.heartbeat_sim.history[0]
            c1,c2,c3,c4,c5,c6 = st.columns(6)
            c1.metric("⏰ 时间", latest['timestamp'])
            c2.metric("📍 纬度", f"{latest['lat']:.6f}")
            c3.metric("📍 经度", f"{latest['lng']:.6f}")
            c4.metric("📊 高度", f"{latest['altitude']}m")
            c5.metric("🔋 电压", f"{latest['voltage']}V")
            c6.metric("🛰️ 卫星", latest['satellites'])
            st.progress(latest['progress'], text=f"飞行进度：{latest['progress']*100:.1f}%")

    # ==================== 障碍物管理 ====================
    elif page == "🚧 障碍物管理":
        st.header("🚧 障碍物管理")
        st.info(f"当前共 {len(st.session_state.obstacles_gcj)} 个障碍物")
        col1, col2 = st.columns([1, 1.5])
        with col1:
            for i, obs in enumerate(st.session_state.obstacles_gcj):
                na, h, btn = st.columns([2,1,1])
                na.write(f"🚧 {obs.get('name', f'障碍物{i+1}')}")
                h.write(f"{obs.get('height',20)}m")
                if btn.button("删除", key=f"del{i}"):
                    st.session_state.obstacles_gcj.pop(i)
                    st.rerun()
            st.columns(2)[0].button("💾 保存到缓存", on_click=save_obstacles_to_cache)
            st.columns(2)[1].button("📂 从缓存加载", on_click=load_obstacles_from_cache)
            if st.button("🗑️ 全部清除"):
                st.session_state.obstacles_gcj = []
                st.rerun()

if __name__ == "__main__":
    main()
