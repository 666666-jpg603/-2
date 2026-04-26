import streamlit as st
import math
import folium
from streamlit_folium import folium_static

# --------------------------
# 基础坐标与工具函数
# --------------------------
def gcj02_to_wgs84(lng, lat):
    # 简易GCJ02转WGS84（适配高德地图）
    pi = 3.1415926535897932384626
    a = 6378245.0
    ee = 0.00669342162296594323
    dlat = transform_lat(lng - 105.0, lat - 35.0)
    dlng = transform_lng(lng - 105.0, lat - 35.0)
    radlat = lat / 180.0 * pi
    magic = math.sin(radlat)
    magic = 1 - ee * magic * magic
    sqrtmagic = math.sqrt(magic)
    dlat = (dlat * 180.0) / ((a * (1 - ee)) / (magic * sqrtmagic) * pi)
    dlng = (dlng * 180.0) / (a / sqrtmagic * math.cos(radlat) * pi)
    return lng - dlng, lat - dlat

def transform_lat(x, y):
    ret = -100.0 + 2.0 * x + 3.0 * y + 0.2 * y * y + 0.1 * x * y + 0.2 * math.sqrt(abs(x))
    ret += (20.0 * math.sin(6.0 * x * math.pi) + 20.0 * math.sin(2.0 * x * math.pi)) * 2.0 / 3.0
    ret += (20.0 * math.sin(y * math.pi) + 40.0 * math.sin(y / 3.0 * math.pi)) * 2.0 / 3.0
    ret += (160.0 * math.sin(y / 12.0 * math.pi) + 320 * math.sin(y * math.pi / 30.0)) * 2.0 / 3.0
    return ret

def transform_lng(x, y):
    ret = 300.0 + x + 2.0 * y + 0.1 * x * x + 0.1 * x * y + 0.1 * math.sqrt(abs(x))
    ret += (20.0 * math.sin(6.0 * x * math.pi) + 20.0 * math.sin(2.0 * x * math.pi)) * 2.0 / 3.0
    ret += (20.0 * math.sin(x * math.pi) + 40.0 * math.sin(x / 3.0 * math.pi)) * 2.0 / 3.0
    ret += (150.0 * math.sin(x / 12.0 * math.pi) + 300.0 * math.sin(x / 30.0 * math.pi)) * 2.0 / 3.0
    return ret

def point_in_polygon(point, polygon):
    """判断点是否在障碍物多边形内"""
    x, y = point
    inside = False
    p1x, p1y = polygon[0]
    for i in range(len(polygon)+1):
        p2x, p2y = polygon[i % len(polygon)]
        if y > min(p1y, p2y):
            if y <= max(p1y, p2y):
                if x <= max(p1x, p2x):
                    if p1y != p2y:
                        xinters = (y-p1y)*(p2x-p1x)/(p2y-p1y)+p1x
                    if p1x == p2x or x <= xinters:
                        inside = not inside
        p1x, p1y = p2x, p2y
    return inside

def distance_point_to_segment(px, py, x1, y1, x2, y2):
    """点到线段最短距离 单位：度"""
    A = px - x1
    B = py - y1
    C = x2 - x1
    D = y2 - y1
    dot = A*C + B*D
    len_sq = C*C + D*D
    param = -1
    if len_sq != 0:
        param = dot / len_sq
    if param < 0:
        xx, yy = x1, y1
    elif param > 1:
        xx, yy = x2, y2
    else:
        xx = x1 + param*C
        yy = y1 + param*D
    return math.hypot(px-xx, py-yy)

# --------------------------
# 核心：障碍物碰撞检测
# --------------------------
def is_path_blocked(start, end, obstacles, flight_height, safe_radius_m):
    """
    检测直线路径是否被障碍物阻挡
    高度判定：飞行高度 < 障碍物高度+安全半径 → 判定阻挡
    """
    meter_per_deg = 111000
    safe_radius_deg = safe_radius_m / meter_per_deg

    for obs in obstacles:
        obs_poly = obs["polygon"]
        obs_height = obs["height"]

        # 高度不足，才需要水平避障
        if flight_height >= obs_height + safe_radius_m:
            continue

        # 遍历障碍物所有顶点，判断航线距离
        for p in obs_poly:
            dist_deg = distance_point_to_segment(p[0], p[1], start[0], start[1], end[0], end[1])
            if dist_deg < safe_radius_deg:
                return True
        
        # 检测线段是否穿过障碍物内部
        mid_point = ((start[0]+end[0])/2, (start[1]+end[1])/2)
        if point_in_polygon(mid_point, obs_poly):
            return True
    return False

# --------------------------
# ✅ 修复后的绕行核心函数
# 保证：起点永远是A，终点永远是B
# --------------------------
def get_perpendicular_offset(start, end, offset_meter, side='left'):
    """计算航线垂直左右偏移量"""
    dx = end[0] - start[0]
    dy = end[1] - start[1]
    line_len = math.hypot(dx, dy)
    if line_len < 1e-9:
        return 0, 0
    
    # 航向单位向量
    ux = dx / line_len
    uy = dy / line_len

    # 垂直方向法向量
    if side == 'left':
        perp_x, perp_y = -uy, ux
    else:
        perp_x, perp_y = uy, -ux

    deg_per_meter = 1 / 111000
    offset_deg = offset_meter * deg_per_meter
    return perp_x * offset_deg, perp_y * offset_deg


def generate_avoidance_path(start_A, end_B, obstacles, flight_height, safe_radius, side='left'):
    # 1. 原始直线路径无遮挡，直接直飞
    if not is_path_blocked(start_A, end_B, obstacles, flight_height, safe_radius):
        return [start_A, end_B]
    
    # 2. 逐级放大偏移，寻找安全绕行路径
    base_offset = safe_radius * 2.5
    max_attempt = 12

    for scale in range(1, max_attempt+1):
        off_x, off_y = get_perpendicular_offset(start_A, end_B, base_offset * scale, side)

        # 偏移后的平行线段端点
        offset_p1 = [start_A[0] + off_x, start_A[1] + off_y]
        offset_p2 = [end_B[0] + off_x, end_B[1] + off_y]

        # 检测绕行段是否安全
        if not is_path_blocked(offset_p1, offset_p2, obstacles, flight_height, safe_radius):
            # ✅ 核心修复：完整路径 精准从A出发，精准回到B
            full_path = [
                start_A,     # 原始起点A
                offset_p1,   # 切入绕行
                offset_p2,   # 平行避障航行
                end_B        # 回归原始终点B
            ]
            return full_path
    
    # 绕行失败兜底，返回原直线
    return [start_A, end_B]

# --------------------------
# 页面初始化与UI
# --------------------------
st.set_page_config(page_title="无人机避障航线规划", layout="wide")
st.title("无人机航线规划 - 自动避障")

# 侧边控制面板
with st.sidebar:
    st.header("控制面板")

    st.subheader("起点 A")
    a_lng = st.number_input("A 经度", value=121.232945, format="%.6f")
    a_lat = st.number_input("A 纬度", value=31.8746956, format="%.7f")
    point_A = (a_lng, a_lat)
    if st.button("设置 A 点"):
        st.success(f"起点A已设置: {point_A}")

    st.subheader("终点 B")
    b_lng = st.number_input("B 经度", value=121.235204, format="%.6f")
    b_lat = st.number_input("B 纬度", value=31.8751589, format="%.7f")
    point_B = (b_lng, b_lat)

    st.divider()
    flight_height = st.slider("飞行高度 (m)", min_value=5, max_value=100, value=10)
    safe_radius = st.slider("安全避让半径 (m)", min_value=5, max_value=100, value=30)
    avoid_side = st.radio("绕行策略", ["向左绕行", "向右绕行"])
    side_flag = "left" if avoid_side == "向左绕行" else "right"

# --------------------------
# 模拟障碍物数据（你图中红色禁飞区）
# --------------------------
default_obstacles = [
    {
        "name": "建筑物障碍区",
        "height": 30, # 障碍物高度30m > 飞行10m，触发避障
        "polygon": [
            [121.2342, 31.8742],
            [121.2358, 31.8738],
            [121.2365, 31.8748],
            [121.2348, 31.8753]
        ]
    }
]

# --------------------------
# 航线计算 + 地图绘制
# --------------------------
st.subheader("规划地图")

# 障碍物预警提示
if is_path_blocked(point_A, point_B, default_obstacles, flight_height, safe_radius):
    st.warning(f"⚠️ 直线航线被建筑物阻挡！当前飞行高度 {flight_height}m，某些障碍物高于此高度+安全半径。")
    # 生成修正后的绕行航线
    fly_path = generate_avoidance_path(point_A, point_B, default_obstacles, flight_height, safe_radius, side_flag)
else:
    st.success("✅ 直线航线安全，无需绕行")
    fly_path = [point_A, point_B]


# 初始化高德底图
m = folium.Map(location=[(a_lat+b_lat)/2, (a_lng+b_lng)/2], zoom_start=16, tiles='https://webrd02.is.autonavi.com/appmaptile?lang=zh_cn&size=1&scale=1&style=8&x={x}&y={y}&z={z}', attr='高德地图')

# 绘制红色障碍物禁飞区
for obs in default_obstacles:
    folium.Polygon(
        locations=[[p[1], p[0]] for p in obs["polygon"]],
        color="red",
        fill=True,
        fill_color="red",
        fill_opacity=0.5
    ).add_to(m)

# 绘制原始危险直线
folium.PolyLine(
    locations=[[point_A[1], point_A[0]], [point_B[1], point_B[0]]],
    color="green",
    weight=3,
    opacity=0.6,
    tooltip="原始直线路线"
).add_to(m)

# 绘制修正后的避障绕行航线
folium.PolyLine(
    locations=[[p[1], p[0]] for p in fly_path],
    color="blue",
    weight=5,
    tooltip="自动避障绕行航线"
).add_to(m)

# 标记起点终点
folium.Marker(location=[point_A[1], point_A[0]], popup="起点A", icon=folium.Icon(color="green")).add_to(m)
folium.Marker(location=[point_B[1], point_B[0]], popup="终点B", icon=folium.Icon(color="red")).add_to(m)

# 渲染地图
folium_static(m, width=900, height=600)

# 路径信息展示
st.markdown(f"""
### 📋 航线信息
- 飞行模式：{'自动侧向避障绕行' if len(fly_path)>2 else '直线通行'}
- 绕行方向：{avoid_side}
- 最终航点数量：{len(fly_path)} 个
- 航点坐标列表：`{fly_path}`
""")
