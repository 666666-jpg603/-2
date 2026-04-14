import streamlit as st
import streamlit.components.v1 as components
import pandas as pd
from datetime import datetime, timedelta
import json
import os
import math
from typing import List, Dict, Tuple

# ==============================================
# 1. 坐标转换工具（GCJ-02 <-> WGS-84）
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
# 2. 障碍物配置工具
# ==============================================
CONFIG_PATH = "obstacle_config.json"

def save_obstacles(obstacles: List[Dict]) -> bool:
    try:
        with open(CONFIG_PATH, "w", encoding="utf-8") as f:
            json.dump(obstacles, f, ensure_ascii=False, indent=4)
        return True
    except:
        return False

def load_obstacles() -> List[Dict]:
    if not os.path.exists(CONFIG_PATH):
        return []
    try:
        with open(CONFIG_PATH, "r", encoding="utf-8") as f:
            return json.load(f)
    except:
        return []

def get_max_obstacle_height(obstacles: List[Dict]) -> float:
    if not obstacles:
        return 0.0
    return max(obs["height"] for obs in obstacles)

# ==============================================
# 3. 心跳包监控工具
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
            return f"连接超时", "error"
        else:
            return "连接正常", "success"

    def get_history_df(self):
        return pd.DataFrame(self.history)

# ==============================================
# 4. 航线规划工具
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
# 5. 主界面初始化
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
# 页面1：航线规划（核心修复：地图渲染）
# ==============================================
if page == "航线规划":
    st.title("✈️ 无人机航线规划系统")
    st.subheader("📊 3D高德地图 · 障碍物圈选 · 智能航线规划")

    col1, col2 = st.columns([1, 3])
    with col1:
        st.subheader("⚙️ 坐标系设置")
        input_coord = st.radio("输入坐标系", ["GCJ-02(高德/百度)", "WGS-84"], 
                              index=0 if st.session_state.input_coord == "GCJ-02" else 1)
        st.session_state.input_coord = "GCJ-02" if "GCJ-02" in input_coord else "WGS-84"

        st.subheader("📍 起点/终点设置")
        st.write("**起点A**")
        a_lat = st.number_input("起点纬度", value=32.2322, format="%.6f")
        a_lng = st.number_input("起点经度", value=118.7490, format="%.6f")
        if st.button("✅ 设置A点"):
            lng_gcj, lat_gcj = convert_coords(a_lng, a_lat, st.session_state.input_coord, "GCJ-02")
            st.session_state.start_point = (lng_gcj, lat_gcj)
            st.success("起点A设置成功！")

        st.write("**终点B**")
        b_lat = st.number_input("终点纬度", value=32.2343, format="%.6f")
        b_lng = st.number_input("终点经度", value=118.7490, format="%.6f")
        if st.button("✅ 设置B点"):
            lng_gcj, lat_gcj = convert_coords(b_lng, b_lat, st.session_state.input_coord, "GCJ-02")
            st.session_state.end_point = (lng_gcj, lat_gcj)
            st.success("终点B设置成功！")

        st.subheader("✈️ 飞行参数设置")
        st.session_state.fly_height = st.slider("设定飞行高度(m)", 10.0, 200.0, st.session_state.fly_height, 1.0)
        st.session_state.safe_radius = st.slider("安全半径(m)", 1.0, 20.0, st.session_state.safe_radius, 1.0)
        st.info(f"安全半径：{st.session_state.safe_radius}m（默认5m）")

        st.subheader("🚧 障碍物管理")
        if st.button("💾 保存障碍物"):
            save_obstacles(st.session_state.obstacles)
            st.success("已保存")
        if st.button("📂 加载障碍物"):
            st.session_state.obstacles = load_obstacles()
            st.success(f"已加载 {len(st.session_state.obstacles)} 个")
        if st.button("🗑️ 清除全部障碍物"):
            st.session_state.obstacles = []
            save_obstacles([])
            st.success("已清空")

        st.subheader("🚀 航线生成")
        if st.button("生成全部航线"):
            if not st.session_state.start_point or not st.session_state.end_point:
                st.error("请先设置起点和终点！")
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
        st.subheader("🗺️ 3D高德卫星地图")
        # 修复1：强制设置默认坐标，避免空值
        start_lng, start_lat = st.session_state.start_point or (118.7490, 32.2322)
        end_lng, end_lat = st.session_state.end_point or (118.7490, 32.2343)
        fly_h = st.session_state.fly_height
        obs_json = json.dumps(st.session_state.obstacles, ensure_ascii=False)
        routes = st.session_state.get("routes", None)
        route_script = ""

        # 修复2：航线JS代码完整拼接
        if routes:
            direct_str = json.dumps([[p[0], p[1]] for p in routes["direct"]["route"]])
            left_str = json.dumps([[p[0], p[1]] for p in routes["left"]["route"]])
            right_str = json.dumps([[p[0], p[1]] for p in routes["right"]["route"]])
            best_str = json.dumps([[p[0], p[1]] for p in routes["best"]["route"]])

            route_script = f"""
            new AMap.Polyline({{path:{direct_str}, strokeColor:'#0969da', strokeWeight:6, height:{fly_h}, map:map}});
            new AMap.Polyline({{path:{left_str}, strokeColor:'#ff7d00', strokeWeight:5, height:{fly_h}, map:map}});
            new AMap.Polyline({{path:{right_str}, strokeColor:'#ffd100', strokeWeight:5, height:{fly_h}, map:map}});
            new AMap.Polyline({{path:{best_str}, strokeColor:'#00b42a', strokeWeight:8, height:{fly_h+3}, map:map}});
            """

        # 修复3：地图HTML完整重构，确保容器正确渲染
        amap_html = f"""
        <!DOCTYPE html>
        <html>
        <head>
            <meta charset="utf-8">
            <style>
                html, body, #container {{
                    width: 100%;
                    height: 700px;
                    margin: 0;
                    padding: 0;
                }}
            </style>
        </head>
        <body>
            <div id="container"></div>
            <script src="https://webapi.amap.com/maps?v=2.0&key=685b1aa7462dd187d2e5c7a79d45a4c0&plugin=AMap.ToolBar"></script>
            <script>
                // 修复4：等待容器加载完成后再初始化地图
                window.onload = function() {{
                    const map = new AMap.Map('container', {{
                        viewMode: '3D',
                        pitch: 55,
                        zoom: 16,
                        center: [{start_lng}, {start_lat}],
                        layers: [AMap.createDefaultLayer({{mapStyle: 'amap://styles/satellite'}})]
                    }});

                    // 起点终点
                    new AMap.Marker({{
                        position: [{start_lng},{start_lat}],
                        content: '<div style="color:red;font-weight:bold;">起点A</div>',
                        map: map
                    }});
                    new AMap.Marker({{
                        position: [{end_lng},{end_lat}],
                        content: '<div style="color:green;font-weight:bold;">终点B</div>',
                        map: map
                    }});

                    // 障碍物
                    const obstacles = {obs_json};
                    obstacles.forEach(obs => {{
                        new AMap.Polygon({{
                            path: obs.poly,
                            strokeColor: '#ff4d4f',
                            fillColor: '#ff4d4f',
                            fillOpacity: 0.5,
                            height: obs.height || 30,
                            map: map
                        }});
                    }});

                    // 航线
                    {route_script}

                    // 工具栏
                    const toolbar = new AMap.ToolBar();
                    map.addControl(toolbar);
                }};
            </script>
        </body>
        </html>
        """
        # 修复5：调整components.html参数，确保渲染
        components.html(amap_html, height=720, scrolling=False, width=None)

    # 航线信息展示
    if routes:
        st.subheader("📋 航线信息")
        c1, c2, c3, c4 = st.columns(4)
        c1.metric("最大障碍物高度", f"{routes['max_obstacle_height']:.1f}m")
        c2.metric("飞行高度", f"{st.session_state.fly_height}m")
        c3.metric("安全半径", f"{st.session_state.safe_radius}m")
        c4.metric("可直接飞跃", "✅ 是" if routes["can_fly_over"] else "❌ 否")

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
# 页面2：飞行监控
# ==============================================
elif page == "飞行监控":
    st.title("📡 无人机飞行监控系统")
    col1, col2 = st.columns(2)
    with col1:
        if st.button("📤 发送心跳包"):
            st.session_state.hb_monitor.send_heartbeat()
            st.success("心跳包发送成功")
    with col2:
        status, typ = st.session_state.hb_monitor.check_status()
        st.success(status) if typ == "success" else st.error(status)

    df = st.session_state.hb_monitor.get_history_df()
    if not df.empty:
        st.line_chart(df.set_index("时间")["序号"])
        st.dataframe(df)
    else:
        st.info("暂无心跳数据")
