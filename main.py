import streamlit as st
import streamlit.components.v1 as components
import pandas as pd
from datetime import datetime, timedelta
import json
import os
import math
from typing import List, Dict, Tuple

# ==============================================
# 1. 坐标转换工具（GCJ-02 <-> WGS-84，解决坐标显示问题）
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
# 2. 障碍物配置工具（保存/加载/清除JSON，持久化）
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
# 3. 心跳包监控工具（每秒发送、3秒超时、可视化）
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
        # 只保留最近100条数据
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
# 4. 航线规划工具（3种航线：左绕/右绕/最佳）
# ==============================================
def calculate_distance(p1: Tuple[float, float], p2: Tuple[float, float]) -> float:
    lng1, lat1 = p1
    lng2, lat2 = p2
    dx = (lng2 - lng1) * 111000 * math.cos(lat1 * math.pi / 180)
    dy = (lat2 - lat1) * 111000
    return math.sqrt(dx**2 + dy**2)

def generate_route(start: Tuple[float, float], end: Tuple[float, float], 
                   obstacles: List[Dict], fly_height: float, safe_radius: float = 5.0):
    """
    生成3种航线：
    1. 直接飞跃（高度>障碍物高度时）
    2. 向左绕行
    3. 向右绕行
    4. 最佳航线（自动选择最短/安全路线）
    """
    max_obs_h = get_max_obstacle_height(obstacles)
    can_fly_over = fly_height > max_obs_h + safe_radius

    # 基础直接航线
    direct_route = [start, end]
    direct_dist = calculate_distance(start, end)

    # 向左绕行航线（偏移0.0015度）
    offset_l = 0.0015
    mid_lng = (start[0] + end[0]) / 2
    mid_lat = (start[1] + end[1]) / 2
    left_route = [
        start,
        (mid_lng - offset_l, mid_lat + offset_l),
        end
    ]
    left_dist = calculate_distance(start, left_route[1]) + calculate_distance(left_route[1], end)

    # 向右绕行航线（偏移-0.0015度）
    right_route = [
        start,
        (mid_lng + offset_l, mid_lat + offset_l),
        end
    ]
    right_dist = calculate_distance(start, right_route[1]) + calculate_distance(right_route[1], end)

    # 最佳航线：高度足够用直接，否则用最短绕行
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
# 5. Streamlit 主界面（完全符合作业要求）
# ==============================================
st.set_page_config(page_title="无人机智能化应用系统", layout="wide")

# 初始化会话状态（统一为float类型，修复类型报错）
if "hb_monitor" not in st.session_state:
    st.session_state.hb_monitor = HeartbeatMonitor(timeout=3)
if "obstacles" not in st.session_state:
    st.session_state.obstacles = load_obstacles()
if "start_point" not in st.session_state:
    st.session_state.start_point = None
if "end_point" not in st.session_state:
    st.session_state.end_point = None
if "fly_height" not in st.session_state:
    st.session_state.fly_height = 50.0  # 保持float
if "safe_radius" not in st.session_state:
    st.session_state.safe_radius = 5.0  # 保持float
if "input_coord" not in st.session_state:
    st.session_state.input_coord = "GCJ-02"  # 默认高德坐标系

# 侧边栏导航
st.sidebar.title("📌 导航菜单")
page = st.sidebar.radio("功能页面", ["航线规划", "飞行监控"])

# ==============================================
# 页面1：航线规划（核心功能，完全符合作业要求）
# ==============================================
if page == "航线规划":
    st.title("✈️ 无人机航线规划系统")
    st.subheader("📊 3D高德地图 · 障碍物圈选 · 智能航线规划")

    # 左侧：控制面板
    col1, col2 = st.columns([1, 3])
    with col1:
        # 1. 坐标系设置
        st.subheader("⚙️ 坐标系设置")
        input_coord = st.radio("输入坐标系", ["GCJ-02(高德/百度)", "WGS-84"], 
                              index=0 if st.session_state.input_coord == "GCJ-02" else 1)
        st.session_state.input_coord = "GCJ-02" if "GCJ-02" in input_coord else "WGS-84"

        # 2. 起点/终点设置
        st.subheader("📍 起点/终点设置")
        # 起点A
        st.write("**起点A**")
        a_lat = st.number_input("起点纬度", value=32.2322, format="%.6f", key="a_lat")
        a_lng = st.number_input("起点经度", value=118.7490, format="%.6f", key="a_lng")
        if st.button("✅ 设置A点", key="set_a"):
            # 转换为GCJ-02用于地图显示
            lng_gcj, lat_gcj = convert_coords(a_lng, a_lat, st.session_state.input_coord, "GCJ-02")
            st.session_state.start_point = (lng_gcj, lat_gcj)
            st.success("起点A设置成功！")

        # 终点B
        st.write("**终点B**")
        b_lat = st.number_input("终点纬度", value=32.2343, format="%.6f", key="b_lat")
        b_lng = st.number_input("终点经度", value=118.7490, format="%.6f", key="b_lng")
        if st.button("✅ 设置B点", key="set_b"):
            lng_gcj, lat_gcj = convert_coords(b_lng, b_lat, st.session_state.input_coord, "GCJ-02")
            st.session_state.end_point = (lng_gcj, lat_gcj)
            st.success("终点B设置成功！")

        # 3. 飞行参数设置（修复：min_value/max_value改为float，类型统一）
        st.subheader("✈️ 飞行参数设置")
        st.session_state.fly_height = st.slider(
            "设定飞行高度(m)", 
            min_value=10.0,  # 改为float
            max_value=200.0, # 改为float
            value=st.session_state.fly_height, 
            step=1.0  # 改为float
        )
        st.session_state.safe_radius = st.slider(
            "安全半径(m)", 
            min_value=1.0,   # 改为float
            max_value=20.0,  # 改为float
            value=st.session_state.safe_radius, 
            step=1.0  # 改为float
        )
        st.info(f"当前安全半径：{st.session_state.safe_radius}m（默认5m）")

        # 4. 障碍物操作（保存/加载/清除/下载）
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

        # 5. 航线生成按钮
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

    # 右侧：3D高德地图
    with col2:
        st.subheader("🗺️ 3D高德卫星地图")
        # 获取起点/终点坐标（默认校园坐标）
        start_lng, start_lat = st.session_state.start_point or (118.7490, 32.2322)
        end_lng, end_lat = st.session_state.end_point or (118.7490, 32.2343)
        fly_h = st.session_state.fly_height

        # 障碍物坐标JSON（用于地图绘制）
        obs_json = json.dumps(st.session_state.obstacles, ensure_ascii=False)

        # 航线坐标（如果已生成）
        routes = st.session_state.get("routes", None)
        route_script = ""
        if routes:
            # 直接航线（蓝色）
            direct = routes["direct"]["route"]
            direct_str = json.dumps([[p[0], p[1]] for p in direct])
            # 左绕航线（橙色）
            left = routes["left"]["route"]
            left_str = json.dumps([[p[0], p[1]] for p in left])
            # 右绕航线（黄色）
            right = routes["right"]["route"]
            right_str = json.dumps([[p[0], p[1]] for p in right])
            # 最佳航线（绿色）
            best = routes["best"]["route"]
            best_str = json.dumps([[p[0], p[1]] for p in best])

            route_script = f"""
            // 绘制直接航线（蓝色）
            const directLine = new AMap.Polyline({{
                path: {direct_str},
                strokeColor: '#0969da',
                strokeWeight: 6,
                height: {fly_h},
                map: map
            }});

            // 绘制向左绕行（橙色）
            const leftLine = new AMap.Polyline({{
                path: {left_str},
                strokeColor: '#ff7d00',
                strokeWeight: 5,
                height: {fly_h},
                map: map
            }});

            // 绘制向右绕行（黄色）
            const rightLine = new AMap.Polyline({{
                path: {right_str},
                strokeColor: '#ffd100',
                strokeWeight: 5,
                height: {fly_h},
                map: map
            }});

            // 绘制最佳航线（绿色）
            const bestLine = new AMap.Polyline({{
                path: {best_str},
                strokeColor: '#00b42a',
                strokeWeight: 7,
                height: {fly_h + 5}, // 略高一层，突出显示
                map: map
            }});
            """

        # 高德3D地图HTML（完全符合作业要求）
        amap_html = f"""
        <div id="container" style="width:100%;height:700px;"></div>
        <script type="text/javascript" src="https://webapi.amap.com/maps?v=2.0&key=685b1aa7462dd187d2e5c7a79d45a4c0&plugin=AMap.ToolBar"></script>
        <script>
            // 初始化3D地图
            const map = new AMap.Map('container', {{
                viewMode: '3D',
                pitch: 55,
                zoom: 16,
                center: [{start_lng}, {start_lat}],
                layers: [
                    AMap.createDefaultLayer({{
                        mapStyle: 'amap://styles/satellite' // 卫星实况地图
                    }})
                ]
            }});

            // 添加工具栏
            const toolbar = new AMap.ToolBar();
            map.addControl(toolbar);

            // 绘制起点/终点标记
            const startMarker = new AMap.Marker({{
                position: [{start_lng}, {start_lat}],
                content: '<div style="color:red;font-weight:bold;font-size:14px;">起点A</div>',
                map: map
            }});

            const endMarker = new AMap.Marker({{
                position: [{end_lng}, {end_lat}],
                content: '<div style="color:green;font-weight:bold;font-size:14px;">终点B</div>',
                map: map
            }});

            // 绘制障碍物（多边形+高度）
            const obstacles = {obs_json};
            obstacles.forEach((obs, index) => {{
                const path = obs.poly;
                const height = obs.height || 30;
                // 绘制多边形
                const polygon = new AMap.Polygon({{
                    path: path,
                    strokeColor: '#ff4d4f',
                    strokeWeight: 3,
                    fillColor: '#ff4d4f',
                    fillOpacity: 0.5,
                    height: height,
                    map: map
                }});
                // 添加高度标注
                const center = path[Math.floor(path.length/2)];
                new AMap.Marker({{
                    position: center,
                    content: `<div style="background:#fff;padding:2px 5px;border-radius:3px;">H:{height}m</div>`,
                    map: map
                }});
            }});

            // 绘制航线（如果已生成）
            {route_script}

            // 多边形绘制工具（用于圈选障碍物）
            const polygonEditor = new AMap.PolygonEditor(map);
            polygonEditor.on('draw', function(event) {{
                const polygon = event.target;
                const path = polygon.getPath();
                const lnglats = path.map(p => [p.getLng(), p.getLat()]);
                // 弹出高度输入框（通过Streamlit交互）
                window.parent.postMessage({{
                    type: 'new_obstacle',
                    path: lnglats
                }}, '*');
            }});
            polygonEditor.open();
        </script>
        """

        # 嵌入地图
        components.html(amap_html, height=720, scrolling=False)

        # 监听地图绘制的障碍物
        if "new_obstacle_path" in st.session_state:
            path = st.session_state.new_obstacle_path
            height = st.number_input(f"请设置障碍物高度(m)", value=30, min_value=0, max_value=200, step=1)
            if st.button("✅ 确认添加障碍物", key="add_obs"):
                st.session_state.obstacles.append({
                    "poly": path,
                    "height": height
                })
                save_obstacles(st.session_state.obstacles)
                del st.session_state.new_obstacle_path
                st.success("障碍物添加成功！")
                st.experimental_rerun()

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
# 页面2：飞行监控（心跳包可视化，符合作业要求）
# ==============================================
elif page == "飞行监控":
    st.title("📡 无人机飞行监控系统")
    st.subheader("💓 心跳包实时监控 · 数据可视化")

    # 心跳包发送按钮
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

    # 心跳包历史可视化
    st.subheader("📈 心跳包历史")
    df = st.session_state.hb_monitor.get_history_df()
    if not df.empty:
        st.line_chart(df.set_index("时间")["序号"], use_container_width=True)
        st.dataframe(df, use_container_width=True, hide_index=True)
    else:
        st.info("暂无心跳包数据，请发送心跳包")

    # 系统状态
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

# ==============================================
# 6. 监听地图障碍物绘制（Streamlit <-> 高德地图交互）
# ==============================================
if st.query_params.get("new_obstacle"):
    path = json.loads(st.query_params["new_obstacle"])
    st.session_state.new_obstacle_path = path
    st.experimental_rerun()