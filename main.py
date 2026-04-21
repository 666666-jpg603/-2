import streamlit as st
import streamlit.components.v1 as components
import pandas as pd
from datetime import datetime, timedelta
import json
import os
import math
from typing import List, Dict, Tuple

# ==============================================
# 1. 坐标转换工具（GCJ-02 <-> WGS-84，完全保留）
# ==============================================
PI = 3.141592653589793
A = 6378245.0
EE = 0.006693421622965943

def _transform_lat(lng, lat):
    ret = -100.0 + 2.0 * lng + 3.0 * lat + 0.2 * lat * lat + 0.1 * lng * lat + 0.2 * math.sqrt(abs(lng))
    ret += (20.0 * math.sin(6.0 * lng * PI) + 20.0 * math.sin(2.0 * lng * PI)) * 2.0 / 3.0
    ret += (20.0 * math.sin(lat * PI) + 40.0 * math.sin(lat / 3.0 * PI)) * 2.0 / 3.0
    ret += (160.0 * math.sin(lat / 12.0 * PI) + 320.0 * math.sin(lat * PI / 30.0)) * 2.0 / 3.0
    return ret

def _transform_lng(lng, lat):
    ret = 300.0 + lng + 2.0 * lat + 0.1 * lng * lng + 0.1 * lng * lat + 0.1 * math.sqrt(abs(lng))
    ret += (20.0 * math.sin(6.0 * lng * PI) + 20.0 * math.sin(2.0 * lng * PI)) * 2.0 / 3.0
    ret += (20.0 * math.sin(lng * PI) + 40.0 * math.sin(lng / 3.0 * PI)) * 2.0 / 3.0
    ret += (150.0 * math.sin(lng / 12.0 * PI) + 300.0 * math.sin(lng / 30.0 * PI)) * 2.0 / 3.0
    return ret

def wgs84_to_gcj02(lng, lat):
    if lng < 72.004 or lng > 137.8347 or lat < 0.8293 or lat > 55.8271:
        return lng, lat
    dlat = _transform_lat(lng - 105.0, lat - 35.0)
    dlng = _transform_lng(lng - 105.0, lat - 35.0)
    radlat = lat / 180.0 * PI
    magic = math.sin(radlat)
    magic = 1 - EE * magic * magic
    sqrtmagic = math.sqrt(magic)
    dlat = (dlat * 180.0) / ((A * (1 - EE)) / (magic * sqrtmagic) * PI)
    dlng = (dlng * 180.0) / (A / sqrtmagic * math.cos(radlat) * PI)
    return lng + dlng, lat + dlat

def gcj02_to_wgs84(lng, lat):
    if lng < 72.004 or lng > 137.8347 or lat < 0.8293 or lat > 55.8271:
        return lng, lat
    dlat = _transform_lat(lng - 105.0, lat - 35.0)
    dlng = _transform_lng(lng - 105.0, lat - 35.0)
    radlat = lat / 180.0 * PI
    magic = math.sin(radlat)
    magic = 1 - EE * magic * magic
    sqrtmagic = math.sqrt(magic)
    dlat = (dlat * 180.0) / ((A * (1 - EE)) / (magic * sqrtmagic) * PI)
    dlng = (dlng * 180.0) / (A / sqrtmagic * math.cos(radlat) * PI)
    mglat = lat + dlat
    mglng = lng + dlng
    return lng * 2 - mglng, lat * 2 - mglat

def convert_coords(lng, lat, from_coord, to_coord):
    if from_coord == to_coord:
        return lng, lat
    if from_coord == "WGS-84" and to_coord == "GCJ-02":
        return wgs84_to_gcj02(lng, lat)
    elif from_coord == "GCJ-02" and to_coord == "WGS-84":
        return gcj02_to_wgs84(lng, lat)
    else:
        raise ValueError("不支持的坐标系转换")

# ==============================================
# 2. 障碍物配置工具（完全保留）
# ==============================================
CONFIG_PATH = "obstacle_config.json"

def save_obstacles(obstacles: List[Dict]) -> bool:
    try:
        with open(CONFIG_PATH, "w", encoding="utf-8") as f:
            json.dump(obstacles, f, ensure_ascii=False, indent=4)
        return True
    except Exception as e:
        st.error(f"保存失败：{str(e)}")
        return False

def load_obstacles() -> List[Dict]:
    if not os.path.exists(CONFIG_PATH):
        return []
    try:
        with open(CONFIG_PATH, "r", encoding="utf-8") as f:
            return json.load(f)
    except Exception as e:
        st.error(f"加载失败：{str(e)}")
        return []

def get_max_obstacle_height(obstacles: List[Dict]) -> float:
    if not obstacles:
        return 0.0
    return max(obs["height"] for obs in obstacles)

# ==============================================
# 3. 心跳包监控工具（完全保留）
# ==============================================
class HeartbeatMonitor:
    def __init__(self, timeout=3):
        self.timeout = timeout
        self.last_heartbeat = None
        self.history = []
        self.sequence = 0

    def send_heartbeat(self):
        self.sequence += 1
        now = datetime.now()
        self.last_heartbeat = now
        self.history.append({
            "序号": self.sequence,
            "时间": now.strftime("%Y-%m-%d %H:%M:%S.%f")[:-3]
        })
        if len(self.history) > 100:
            self.history.pop(0)

    def check_status(self):
        if not self.last_heartbeat:
            return "未连接", "error"
        delta = (datetime.now() - self.last_heartbeat).total_seconds()
        if delta > self.timeout:
            return f"连接超时（{delta:.1f}秒）", "error"
        else:
            return "连接正常", "success"

    def get_history_df(self):
        return pd.DataFrame(self.history)

# ==============================================
# 4. 航线规划工具（完全保留）
# ==============================================
def calculate_distance(p1: Tuple[float, float], p2: Tuple[float, float]) -> float:
    lng1, lat1 = p1
    lng2, lat2 = p2
    dx = (lng2 - lng1) * 111000 * math.cos(lat1 * math.pi / 180)
    dy = (lat2 - lat1) * 111000
    return math.sqrt(dx**2 + dy**2)

def generate_route(start: Tuple[float, float], end: Tuple[float, float], 
                   obstacles: List[Dict], fly_height: float, safe_radius: float = 5.0):
    max_obs_h = get_max_obstacle_height(obstacles)
    can_fly_over = fly_height > max_obs_h + safe_radius

    direct_route = [start, end]
    direct_dist = calculate_distance(start, end)

    offset_l = 0.0015
    mid_lng = (start[0] + end[0]) / 2
    mid_lat = (start[1] + end[1]) / 2

    left_route = [start, (mid_lng - offset_l, mid_lat + offset_l), end]
    left_dist = calculate_distance(start, left_route[1]) + calculate_distance(left_route[1], end)

    right_route = [start, (mid_lng + offset_l, mid_lat + offset_l), end]
    right_dist = calculate_distance(start, right_route[1]) + calculate_distance(right_route[1], end)

    if can_fly_over:
        best_route = direct_route
        best_dist = direct_dist
    else:
        best_route = left_route if left_dist < right_dist else right_route
        best_dist = min(left_dist, right_dist)

    return {
        "can_fly_over": can_fly_over,
        "max_obstacle_height": max_obs_h,
        "direct": {"route": direct_route, "distance": direct_dist},
        "left": {"route": left_route, "distance": left_dist},
        "right": {"route": right_route, "distance": right_dist},
        "best": {"route": best_route, "distance": best_dist}
    }

# ==============================================
# 5. 主界面初始化（完全保留）
# ==============================================
st.set_page_config(page_title="无人机智能化应用系统", layout="wide")

if "hb_monitor" not in st.session_state:
    st.session_state.hb_monitor = HeartbeatMonitor(timeout=3)
if "obstacles" not in st.session_state:
    st.session_state.obstacles = load_obstacles()
if "start_point" not in st.session_state:
    st.session_state.start_point = None
if "end_point" not in st.session_state:
    st.session_state.end_point = None
if "fly_height" not in st.session_state:
    st.session_state.fly_height = 50.0
if "safe_radius" not in st.session_state:
    st.session_state.safe_radius = 5.0
if "input_coord" not in st.session_state:
    st.session_state.input_coord = "GCJ-02"

st.sidebar.title("📌 导航菜单")
page = st.sidebar.radio("功能页面", ["航线规划", "飞行监控"])

# ==============================================
# 页面1：航线规划（仅修改起点、终点默认值）
# ==============================================
if page == "航线规划":
    st.title("✈️ 无人机航线规划系统")
    st.subheader("📊 Leaflet开源地图 · 障碍物圈选 · 智能航线规划")

    col1, col2 = st.columns([1, 3])
    with col1:
        st.subheader("⚙️ 坐标系设置")
        input_coord = st.radio("输入坐标系", ["GCJ-02(高德/百度)", "WGS-84"], 
                              index=0 if st.session_state.input_coord == "GCJ-02" else 1)
        st.session_state.input_coord = "GCJ-02" if "GCJ-02" in input_coord else "WGS-84"

        st.subheader("📍 起点/终点设置")
        st.write("**起点A**")
        a_lat = st.number_input("起点纬度", value=32.232945, format="%.6f")
        a_lng = st.number_input("起点经度", value=118.746956, format="%.6f")
        
        if st.button("✅ 设置A点"):
            lng_gcj, lat_gcj = convert_coords(a_lng, a_lat, st.session_state.input_coord, "GCJ-02")
            st.session_state.start_point = (lng_gcj, lat_gcj)
            st.success("起点A设置成功！")

        st.write("**终点B**")
        b_lat = st.number_input("终点纬度", value=32.235204, format="%.6f")
        b_lng = st.number_input("终点经度", value=118.751589, format="%.6f")
        
        if st.button("✅ 设置B点"):
            lng_gcj, lat_gcj = convert_coords(b_lng, b_lat, st.session_state.input_coord, "GCJ-02")
            st.session_state.end_point = (lng_gcj, lat_gcj)
            st.success("终点B设置成功！")

        st.subheader("✈️ 飞行参数设置")
        st.session_state.fly_height = st.slider("设定飞行高度(m)", 10.0, 200.0, st.session_state.fly_height, 1.0)
        st.session_state.safe_radius = st.slider("安全半径(m)", 1.0, 20.0, st.session_state.safe_radius, 1.0)
        st.info(f"安全半径：{st.session_state.safe_radius}m（默认5m）")

        st.subheader("🚧 障碍物管理")
        col_save, col_load = st.columns(2)
        with col_save:
            if st.button("💾 保存到JSON", key="save_obs"):
                if save_obstacles(st.session_state.obstacles):
                    st.success("障碍物已保存！")
        with col_load:
            if st.button("📂 从JSON加载", key="load_obs"):
                st.session_state.obstacles = load_obstacles()
                st.success(f"已加载{len(st.session_state.obstacles)}个障碍物！")

        col_clear, col_download = st.columns(2)
        with col_clear:
            if st.button("🗑️ 清除全部", key="clear_obs"):
                st.session_state.obstacles = []
                save_obstacles([])
                st.success("已清除全部障碍物！")
        with col_download:
            if st.button("📥 下载JSON", key="download_obs"):
                json_str = json.dumps(st.session_state.obstacles, ensure_ascii=False, indent=4)
                st.download_button(
                    label="点击下载",
                    data=json_str,
                    file_name="obstacle_config.json",
                    mime="application/json"
                )

        st.info(f"当前共{len(st.session_state.obstacles)}个障碍物")

        st.subheader("🚀 航线生成")
        if st.button("生成全部航线", key="gen_routes"):
            if not st.session_state.start_point or not st.session_state.end_point:
                st.error("请先设置起点A和终点B！")
            else:
                st.session_state.routes = generate_route(
                    st.session_state.start_point,
                    st.session_state.end_point,
                    st.session_state.obstacles,
                    st.session_state.fly_height,
                    st.session_state.safe_radius
                )
                st.success("航线生成成功！")

    with col2:
        st.subheader("🗺️ Leaflet开源卫星地图（仿目标样式）")
        start_lng, start_lat = st.session_state.start_point or (118.746956, 32.232945)
        end_lng, end_lat = st.session_state.end_point or (118.751589, 32.235204)
        
        obs_json = json.dumps(st.session_state.obstacles, ensure_ascii=False)
        routes = st.session_state.get("routes", None)
        route_script = ""
        obs_script = ""

        # 障碍物渲染脚本
        obs_script = f"""
        const obstacles = {obs_json};
        obstacles.forEach(obs => {{
            L.polygon(obs.poly.map(p => [p[1], p[0]]), {{
                color: 'red',
                fillColor: '#f03',
                fillOpacity: 0.5,
                weight: 3
            }}).addTo(map);
        }});
        """

        # 航线渲染脚本
        if routes:
            best = routes["best"]["route"]
            best_latLng = [[p[1], p[0]] for p in best]
            best_str = json.dumps(best_latLng)

            route_script = f"""
            // 主航线
            L.polyline({best_str}, {{
                color: 'blue',
                weight: 6,
                dashArray: '20, 10',
                opacity: 0.8
            }}).addTo(map);

            // 起点/终点标记
            L.marker([{best[0][1]}, {best[0][0]}], {{icon: L.icon({{
                iconUrl: 'https://raw.githubusercontent.com/pointhi/leaflet-color-markers/master/img/marker-icon-2x-red.png',
                shadowUrl: 'https://cdnjs.cloudflare.com/ajax/libs/leaflet/0.7.7/images/marker-shadow.png',
                iconSize: [25, 41],
                iconAnchor: [12, 41],
                popupAnchor: [1, -34],
                shadowSize: [41, 41]
            }})}}).addTo(map).bindPopup('起点A');

            L.marker([{best[-1][1]}, {best[-1][0]}], {{icon: L.icon({{
                iconUrl: 'https://raw.githubusercontent.com/pointhi/leaflet-color-markers/master/img/marker-icon-2x-green.png',
                shadowUrl: 'https://cdnjs.cloudflare.com/ajax/libs/leaflet/0.7.7/images/marker-shadow.png',
                iconSize: [25, 41],
                iconAnchor: [12, 41],
                popupAnchor: [1, -34],
                shadowSize: [41, 41]
            }})}}).addTo(map).bindPopup('终点B');

            // 播放按钮
            L.control.custom({{
                position: 'bottomleft',
                content: '<button style="background:#2ecc71;color:white;border:none;border-radius:50%;width:40px;height:40px;font-size:20px;cursor:pointer;">▶</button>',
                classes: 'play-button'
            }}).addTo(map);
            """

        # Leaflet地图HTML
        leaflet_html = f"""
        <!DOCTYPE html>
        <html>
        <head>
            <meta charset="utf-8">
            <link rel="stylesheet" href="https://unpkg.com/leaflet@1.9.4/dist/leaflet.css" crossorigin=""/>
            <script src="https://unpkg.com/leaflet@1.9.4/dist/leaflet.js" crossorigin=""></script>
            <script src="https://cdn.jsdelivr.net/npm/leaflet-control-custom/Leaflet.Control.Custom.js"></script>
            <style>
                html, body, #map {{
                    width: 100%;
                    height: 700px;
                    margin: 0;
                    padding: 0;
                }}
                .play-button {{
                    background: transparent;
                    border: none;
                    box-shadow: none;
                }}
            </style>
        </head>
        <body>
            <div id="map"></div>
            <script>
                const map = L.map('map').setView([{start_lat}, {start_lng}], 16);

                L.tileLayer('https://server.arcgisonline.com/ArcGIS/rest/services/World_Imagery/MapServer/tile/{{z}}/{{y}}/{{x}}', {{
                    attribution: 'Tiles &copy; Esri',
                    maxZoom: 18
                }}).addTo(map);

                L.control.zoom({{position: 'topleft'}}).addTo(map);
                L.control.scale({{position: 'bottomleft'}}).addTo(map);

                {obs_script}
                {route_script}

                L.control.attribution({{
                    prefix: 'Leaflet | © OpenStreetMap',
                    position: 'bottomright'
                }}).addTo(map);
            </script>
        </body>
        </html>
        """
        components.html(leaflet_html, height=720, scrolling=False, width=None)

    # 航线信息展示
    if routes:
        st.subheader("📋 航线信息")
        col_info1, col_info2, col_info3, col_info4 = st.columns(4)
        with col_info1:
            st.metric("最大障碍物高度", f"{routes['max_obstacle_height']:.1f}m")
        with col_info2:
            st.metric("飞行高度", f"{st.session_state.fly_height}m")
        with col_info3:
            st.metric("安全半径", f"{st.session_state.safe_radius}m")
        with col_info4:
            st.metric("可直接飞跃", "✅ 是" if routes["can_fly_over"] else "❌ 否")

        st.subheader("📊 航线距离对比")
        dist_df = pd.DataFrame({
            "航线类型": ["直接飞跃", "向左绕行", "向右绕行", "最佳航线"],
            "距离(m)": [
                round(routes["direct"]["distance"], 2),
                round(routes["left"]["distance"], 2),
                round(routes["right"]["distance"], 2),
                round(routes["best"]["distance"], 2)
            ]
        })
        st.dataframe(dist_df, use_container_width=True, hide_index=True)

# ==============================================
# 页面2：飞行监控（完全保留）
# ==============================================
elif page == "飞行监控":
    st.title("📡 无人机飞行监控系统")
    st.subheader("💓 心跳包实时监控 · 数据可视化")

    col1, col2 = st.columns(2)
    with col1:
        if st.button("📤 发送心跳包", key="send_hb"):
            st.session_state.hb_monitor.send_heartbeat()
            st.success("心跳包发送成功！")
    with col2:
        status, status_type = st.session_state.hb_monitor.check_status()
        if status_type == "success":
            st.success(f"✅ {status}")
        else:
            st.error(f"❌ {status}")

    st.subheader("📈 心跳包历史")
    df = st.session_state.hb_monitor.get_history_df()
    if not df.empty:
        st.line_chart(df.set_index("时间")["序号"], use_container_width=True)
        st.dataframe(df, use_container_width=True, hide_index=True)
    else:
        st.info("暂无心跳包数据，请发送心跳包")

    st.subheader("ℹ️ 系统状态")
    col_status1, col_status2 = st.columns(2)
    with col_status1:
        st.metric("已发送心跳包数量", len(st.session_state.hb_monitor.history))
    with col_status2:
        last_hb = st.session_state.hb_monitor.last_heartbeat
        if last_hb:
            st.metric("最后一次心跳时间", last_hb.strftime("%H:%M:%S"))
        else:
            st.metric("最后一次心跳时间", "无")
