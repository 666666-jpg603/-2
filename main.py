import streamlit as st
import folium
from streamlit_folium import folium_static, st_folium
from folium import plugins
import random
import time
import math
import copy
import heapq

# ==================== 页面配置 ====================
st.set_page_config(page_title="无人机地面站系统 - 航线精简+顺滑优化", layout="wide")

# ==================== 坐标常量 ====================
SCHOOL_CENTER_GCJ = [118.7490, 32.2340]
DEFAULT_A_GCJ = [118.746956, 32.232945]
DEFAULT_B_GCJ = [118.751589, 32.235204]

GAODE_SATELLITE_URL_ALT = "https://webst01.is.autonavi.com/appmaptile?style=6&x={x}&y={y}&z={z}"
GAODE_VECTOR_URL = "https://webrd02.is.autonavi.com/appmaptile?lang=zh_cn&size=1&scale=1&style=8&x={x}&y={y}&z={z}"

# ==================== 坐标转换 ====================
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

# ==================== 几何工具 + 路径抽稀（关键修复） ====================
def distance(p1, p2):
    return math.hypot(p1[0]-p2[0], p1[1]-p2[1])

def point_line_dist(pt, a, b):
    if a == b:
        return distance(pt, a)
    return abs((b[1]-a[1])*pt[0] - (b[0]-a[0])*pt[1] + b[0]*a[1] - b[1]*a[0]) / distance(a,b)

# 道格拉斯-普克 路径抽稀，删掉多余航点
def douglas_peuck(path, eps=0.00005):
    if len(path) <= 2:
        return path
    dmax = 0.0
    idx = 0
    start = path[0]
    end = path[-1]
    for i in range(1, len(path)-1):
        d = point_line_dist(path[i], start, end)
        if d > dmax:
            dmax = d
            idx = i
    if dmax > eps:
        left = douglas_peuck(path[:idx+1], eps)
        right = douglas_peuck(path[idx:], eps)
        return left[:-1] + right
    else:
        return [start, end]

# 轻量平滑：不增加航点，只做切线顺滑
def smooth_path_light(path):
    if len(path) <= 2:
        return path
    res = [path[0]]
    for i in range(1, len(path)-1):
        x = (path[i-1][0] + path[i][0] + path[i+1][0]) / 3.0
        y = (path[i-1][1] + path[i][1] + path[i+1][1]) / 3.0
        res.append([x,y])
    res.append(path[-1])
    return res

def point_in_polygon(point, polygon):
    x, y = point
    inside = False
    n = len(polygon)
    for i in range(n):
        x1,y1 = polygon[i]
        x2,y2 = polygon[(i+1)%n]
        if ((y1>y) != (y2>y)) and (x < (x2-x1)*(y-y1)/(y2-y1)+x1):
            inside = not inside
    return inside

def segments_intersect(p1,p2,p3,p4):
    def ccw(A,B,C):
        return (C[1]-A[1])*(B[0]-A[0]) > (B[1]-A[1])*(C[0]-A[0])
    return ccw(p1,p3,p4)!=ccw(p2,p3,p4) and ccw(p1,p2,p3)!=ccw(p1,p2,p4)

def line_intersects_polygon(p1,p2,poly):
    if point_in_polygon(p1,poly) or point_in_polygon(p2,poly):
        return True
    n=len(poly)
    for i in range(n):
        p3=poly[i]
        p4=poly[(i+1)%n]
        if segments_intersect(p1,p2,p3,p4):
            return True
    return False

# ==================== 障碍物阻挡判断 ====================
def is_obstacle_blocking(obs, flight_height, safe_radius):
    obs_height = obs.get('height',20)
    return flight_height <= obs_height + safe_radius

def is_path_blocked(p1,p2,obstacles,flight_height,safe_radius):
    for obs in obstacles:
        poly = obs.get('polygon',[])
        if len(poly)<3:
            continue
        if is_obstacle_blocking(obs,flight_height,safe_radius):
            if line_intersects_polygon(p1,p2,poly):
                return True
    return False

# ==================== 平行偏移绕行（少航点版） ====================
def generate_parallel_offset_path(start,end,obstacles,flight_height,safe_radius,side='left'):
    lng1,lat1 = start
    lng2,lat2 = end
    dx = lng2-lng1
    dy = lat2-lat1
    L = math.hypot(dx,dy)
    if L<1e-7:
        return None

    if side=="left":
        nx = -dy/L
        ny = dx/L
    else:
        nx = dy/L
        ny = -dx/L

    offset_meter = safe_radius * 6.0
    offset_deg = offset_meter / 111000.0
    seg_num = 8   # 直接减少中间点，原生就少航点

    for scale in [1,1.5,2,3]:
        off_deg = offset_deg * scale
        mid = []
        for i in range(1,seg_num):
            t = i/seg_num
            clng = lng1 + dx*t
            clat = lat1 + dy*t
            olng = clng + nx*off_deg
            olat = clat + ny*off_deg
            mid.append([olng,olat])
        path = [start] + mid + [end]
        ok=True
        for i in range(len(path)-1):
            if is_path_blocked(path[i],path[i+1],obstacles,flight_height,safe_radius):
                ok=False
                break
        if ok:
            # 先抽稀 再轻平滑，不增加航点
            path_sparse = douglas_peuck(path, eps=0.00006)
            path_smooth = smooth_path_light(path_sparse)
            return path_smooth
    return None

# ==================== A* 路径规划（精简航点版） ====================
def astar_path(start,end,obstacles,flight_height,safe_radius):
    nodes = [start,end]
    safety = safe_radius / 111000.0 * 2.0

    for obs in obstacles:
        if not is_obstacle_blocking(obs,flight_height,safe_radius):
            continue
        poly = obs.get('polygon',[])
        if len(poly)<3:
            continue
        for i,(x,y) in enumerate(poly):
            prev = poly[(i-1)%len(poly)]
            nextp = poly[(i+1)%len(poly)]
            dx1 = -(y-prev[1])
            dy1 = x-prev[0]
            l1 = math.hypot(dx1,dy1)
            if l1>1e-8:
                dx1/=l1; dy1/=l1
            nx1 = x+dx1*safety
            ny1 = y+dy1*safety

            dx2 = -(nextp[1]-y)
            dy2 = nextp[0]-x
            l2 = math.hypot(dx2,dy2)
            if l2>1e-8:
                dx2/=l2; dy2/=l2
            nx2 = x+dx2*safety
            ny2 = y+dy2*safety
            nodes.append([nx1,ny1])
            nodes.append([nx2,ny2])

    unique_nodes=[]
    for n in nodes:
        exist=False
        for u in unique_nodes:
            if abs(n[0]-u[0])<1e-6 and abs(n[1]-u[1])<1e-6:
                exist=True;break
        if not exist:
            unique_nodes.append(n)

    graph = {i:[] for i in range(len(unique_nodes))}
    for i in range(len(unique_nodes)):
        for j in range(len(unique_nodes)):
            if i==j:continue
            if not is_path_blocked(unique_nodes[i],unique_nodes[j],obstacles,flight_height,safe_radius):
                graph[i].append((j,distance(unique_nodes[i],unique_nodes[j])))

    start_i=end_i=-1
    for i,n in enumerate(unique_nodes):
        if abs(n[0]-start[0])<1e-6 and abs(n[1]-start[1])<1e-6:
            start_i=i
        if abs(n[0]-end[0])<1e-6 and abs(n[1]-end[1])<1e-6:
            end_i=i
    if start_i==-1 or end_i==-1:
        return [start,end]

    open_heap=[]
    import heapq
    heapq.heappush(open_heap,(0,start_i))
    came_from={}
    g_score={i:float('inf') for i in range(len(unique_nodes))}
    g_score[start_i]=0
    f_score={i:float('inf') for i in range(len(unique_nodes))}
    f_score[start_i]=distance(unique_nodes[start_i],unique_nodes[end_i])

    while open_heap:
        current_f,cur = heapq.heappop(open_heap)
        if cur==end_i:
            path=[]
            while cur in came_from:
                path.append(unique_nodes[cur])
                cur = came_from[cur]
            path.append(unique_nodes[start_i])
            path.reverse()
            # 关键：A*路径自动抽稀+轻平滑，航点大幅减少
            path_sparse = douglas_peuck(path, eps=0.00006)
            path_smooth = smooth_path_light(path_sparse)
            return path_smooth
        for neighbor,w in graph[cur]:
            new_g = g_score[cur]+w
            if new_g < g_score[neighbor]:
                came_from[neighbor]=cur
                g_score[neighbor]=new_g
                f_score[neighbor]=new_g + distance(unique_nodes[neighbor],unique_nodes[end_i])
                heapq.heappush(open_heap,(f_score[neighbor],neighbor))
    return [start,end]

def create_avoidance_path(start,end,obstacles,flight_height,safe_radius,strategy):
    if not is_path_blocked(start,end,obstacles,flight_height,safe_radius):
        return [start,end]
    if strategy=='left':
        p=generate_parallel_offset_path(start,end,obstacles,flight_height,safe_radius,'left')
        if p:return p
        p=generate_parallel_offset_path(start,end,obstacles,flight_height,safe_radius,'right')
        if p:return p
        return astar_path(start,end,obstacles,flight_height,safe_radius)
    elif strategy=='right':
        p=generate_parallel_offset_path(start,end,obstacles,flight_height,safe_radius,'right')
        if p:return p
        p=generate_parallel_offset_path(start,end,obstacles,flight_height,safe_radius,'left')
        if p:return p
        return astar_path(start,end,obstacles,flight_height,safe_radius)
    else:
        return astar_path(start,end,obstacles,flight_height,safe_radius)

# ==================== 障碍物、飞行模拟器、地图绘制（保留不变） ====================
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

class HeartbeatSimulator:
    def __init__(self, start_point_gcj):
        self.history = []
        self.current_pos = start_point_gcj.copy()
        self.path = [start_point_gcj.copy()]
        self.path_index = 0
        self.simulating = False
        self.paused = False
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
        self.paused = False
        self.progress = 0.0
        self.distance_traveled = 0.0
        self.total_distance = sum(distance(path[i],path[i+1]) for i in range(len(path)-1))

    def pause(self):
        self.paused = True
    def resume(self):
        self.paused = False
    def stop(self):
        self.simulating = False
    def reset(self):
        self.path_index = 0
        self.current_pos = self.path[0].copy()
        self.progress = 0.0
        self.distance_traveled = 0.0
        self.history = []

    def update_and_generate(self):
        if self.simulating and not self.paused and self.path_index < len(self.path)-1:
            target = self.path[self.path_index+1]
            dx = target[0] - self.current_pos[0]
            dy = target[1] - self.current_pos[1]
            dist = math.hypot(dx,dy)
            step = 0.0002 + (self.speed/100)*0.0006
            if dist < step:
                self.current_pos = target.copy()
                self.path_index += 1
            else:
                ratio = step/dist
                self.current_pos[0] += dx*ratio
                self.current_pos[1] += dy*ratio
            self.distance_traveled += step
            if self.total_distance>0:
                self.progress = min(1.0, self.distance_traveled/self.total_distance)
            if self.path_index >= len(self.path)-1:
                self.simulating = False
                self.progress = 1.0
        else:
            self.simulating = False
            self.progress = 1.0
        return {
            "lng":self.current_pos[0],"lat":self.current_pos[1],
            "progress":self.progress,"current_waypoint":self.path_index+1,"total_waypoints":len(self.path)
        }

def create_planning_map(center_gcj,points_gcj,obstacles_gcj,flight_history=None,planned_path=None,map_type="satellite",straight_blocked=True,safe_radius=5):
    tiles = GAODE_SATELLITE_URL_ALT if map_type=="satellite" else GAODE_VECTOR_URL
    m = folium.Map(location=[center_gcj[1],center_gcj[0]],zoom_start=16,tiles=tiles)
    draw = plugins.Draw(export=True,position='topleft',
        draw_options={'polygon':{'color':'red','fillColor':'red','fillOpacity':0.4},
        'polyline':False,'rectangle':False,'circle':False,'marker':False})
    m.add_child(draw)

    safe_offset = safe_radius / 111000.0
    for obs in obstacles_gcj:
        poly = obs.get('polygon',[])
        if len(poly)<3:continue
        folium.Polygon([[c[1],c[0]] for c in poly],color="red",weight=3,fill=True,fill_color="red",fill_opacity=0.4).add_to(m)

    if points_gcj.get('A'):
        folium.Marker([points_gcj['A'][1],points_gcj['A'][0]],icon=folium.Icon(color="green")).add_to(m)
    if points_gcj.get('B'):
        folium.Marker([points_gcj['B'][1],points_gcj['B'][0]],icon=folium.Icon(color="red")).add_to(m)

    if planned_path and len(planned_path)>1:
        locs = [[p[1],p[0]] for p in planned_path]
        folium.PolyLine(locs,color="green",weight=5).add_to(m)

    return m

# ==================== 主程序 ====================
def main():
    st.title("🏫 无人机地面站 - 航点精简+顺滑优化版")
    if "points_gcj" not in st.session_state:
        st.session_state.points_gcj = {'A':DEFAULT_A_GCJ.copy(),'B':DEFAULT_B_GCJ.copy()}
    if "obstacles_gcj" not in st.session_state:
        st.session_state.obstacles_gcj = []
    if "heartbeat_sim" not in st.session_state:
        st.session_state.heartbeat_sim = HeartbeatSimulator(st.session_state.points_gcj['A'])
    if "flight_altitude" not in st.session_state:
        st.session_state.flight_altitude = 50
    if "planned_path" not in st.session_state:
        st.session_state.planned_path = None

    st.sidebar.subheader("参数设置")
    safe_radius = st.sidebar.number_input("安全半径(m)",1,30,5)
    flight_alt = st.sidebar.number_input("飞行高度(m)",0,200,50)
    st.session_state.flight_altitude = flight_alt
    strategy = st.sidebar.radio("绕行策略",["最佳航线 (A*)","向左绕行","向右绕行"])
    sel = {"最佳航线 (A*)":"best","向左绕行":"left","向右绕行":"right"}[strategy]

    straight_blocked = is_path_blocked(
        st.session_state.points_gcj['A'],st.session_state.points_gcj['B'],
        st.session_state.obstacles_gcj,flight_alt,safe_radius
    )

    if st.sidebar.button("重新规划"):
        st.session_state.planned_path = create_avoidance_path(
            st.session_state.points_gcj['A'],st.session_state.points_gcj['B'],
            st.session_state.obstacles_gcj,flight_alt,safe_radius,sel
        )
        st.rerun()

    # 简单布局
    col1,col2 = st.columns([1,2])
    with col1:
        st.info(f"直线：{'被阻挡' if straight_blocked else '畅通'}")
        st.write("起点终点可在地图圈选障碍物")
    with col2:
        if st.session_state.planned_path is None:
            st.session_state.planned_path = create_avoidance_path(
                st.session_state.points_gcj['A'],st.session_state.points_gcj['B'],
                st.session_state.obstacles_gcj,flight_alt,safe_radius,sel
            )
        m = create_planning_map(SCHOOL_CENTER_GCJ,st.session_state.points_gcj,
                                st.session_state.obstacles_gcj,None,st.session_state.planned_path,
                                "satellite",straight_blocked,safe_radius)
        folium_static(m,width=800,height=550)

if __name__ == "__main__":
    main()
