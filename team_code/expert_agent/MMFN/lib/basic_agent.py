# Copyright (c) # Copyright (c) 2018-2020 CVC.
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

"""
This module implements an agent that roams around a track following random
waypoints and avoiding other vehicles. The agent also responds to traffic lights.
It can also make use of the global route planner to follow a specifed route
"""

import carla
from enum import Enum
from shapely.geometry import Polygon

from .local_planner import LocalPlanner
from .global_planner import GlobalRoutePlanner
from .route_planner import RoutePlanner, PIDController, VehiclePIDController
from .misc import get_speed, is_within_distance, get_trafficlight_trigger_location, compute_distance
import numpy as np

class BasicAgent(object):
    """
    BasicAgent implements an agent that navigates the scene.
    This agent respects traffic lights and other vehicles, but ignores stop signs.
    It has several functions available to specify the route that the agent must follow,
    as well as to change its parameters in case a different driving mode is desired.
    """

    def __init__(self, vehicle, config, interpolate = True, target_speed=20, debug=False):
        """
        Initialization the agent paramters, the local and the global planner.

            :param vehicle: actor to apply to agent logic onto
            :param target_speed: speed (in Km/h) at which the vehicle will move
            :param opt_dict: dictionary in case some of its parameters want to be changed.
                This also applies to parameters related to the LocalPlanner.
        """
        self.config = config
        self.debug = debug
        self._vehicle = vehicle
        self._world = self._vehicle.get_world()
        self._map = self._world.get_map()
        self._ego_velocity = self._vehicle.get_velocity()
        self.interpolate = interpolate # 是否需要插值路径点，一般leaderboard给的点基本够

        self._last_traffic_light = None

        # Base parameters
        self._target_speed = target_speed
        self._base_tlight_threshold = 5.0  # meters
        self._sampling_resolution = 2.0
        self._max_brake = 1.0
        self.STOP_THRESHOLD = 8
        
        # 根据道路宽度和交叉路口的半径来改变
        self.too_close_dis = 999
        self._over_time = False
        self.near_by_dis = 30

        # 最好留出余地
        self._distance_between_change_lane = 8
        self.close_obs_speed_threshold = 1
        self.max_throttle = 0.8
        self.speed_delta = 0.8
        self.consider_angle = 120
        self.red_angle_diff = 10
        self.speed_precentage = 70 # 

        # 沿道路宽度多少以内为直线方向，同时这个参数与 前方路径点 是否有障碍物相关
        self.precentage_of_lane_staright = 0.7

        # Initialize the planners
        if self.interpolate:
            self._local_planner = LocalPlanner(self._vehicle)
            self._global_planner = GlobalRoutePlanner(self._map, self._sampling_resolution)
        else:
            self._dt = 1.0/10.0
            self._near_planner = RoutePlanner(pop_distance = 4.0)
            self._far_planner = RoutePlanner(pop_distance = 8.0)
            self._args_lateral_dict = {'K_P': 1.95, 'K_I': 0.05, 'K_D': 0.2, 'dt': self._dt}
            self._args_longitudinal_dict = {'K_P': 1.0, 'K_I': 0.05, 'K_D': 0, 'dt': self._dt}
            self._vehicle_controller = VehiclePIDController(self._vehicle,
                                                        args_lateral=self._args_lateral_dict,
                                                        args_longitudinal=self._args_longitudinal_dict)

        # Initialize the info
        self._prev_lane_id = self._map.get_waypoint(self._vehicle.get_transform().location).lane_id
        self._prev_road_id = self._map.get_waypoint(self._vehicle.get_transform().location).road_id

        self._near_object = {"vehicle":[], "walker": [], "stop": [], "car_infront": [], "behind": [], "tl": []}

        # Red light detection from SEED
        # Coordinates of the center of the red light detector bounding box. In local coordinates of the vehicle, units are meters
        self.center_bb_light_x = -2.0
        self.center_bb_light_y = 0.0
        self.center_bb_light_z = 0.0

        # Extent of the red light detector bounding box. In local coordinates of the vehicle, units are meters. Size are half of the bounding box
        self.extent_bb_light_x = 4.5
        self.extent_bb_light_y = 1.5
        self.extent_bb_light_z = 2.0

    def set_destination(self, end_location, start_location=None):
        """
        This method creates a list of waypoints between a starting and ending location,
        based on the route returned by the global router, and adds it to the local planner.
        If no starting location is passed, the vehicle local planner's target location is chosen,
        which corresponds (by default), to a location about 5 meters in front of the vehicle.

            :param end_location (carla.Location): final location of the route
            :param start_location (carla.Location): starting location of the route
        """
        if not start_location:
            start_location = self._local_planner.target_waypoint.transform.location
            clean_queue = True
        else:
            start_location = self._vehicle.get_location()
            clean_queue = False

        start_waypoint = self._map.get_waypoint(start_location)
        end_waypoint = self._map.get_waypoint(end_location)

        route_trace = self.trace_route(start_waypoint, end_waypoint)
        self._local_planner.set_global_plan(route_trace, clean_queue=clean_queue)

    def set_global_plan(self, plan, stop_waypoint_creation=True, clean_queue=True):
        """
        Adds a specific plan to the agent.

            :param plan: list of [carla.Location] representing the route to be followed // 
            :param stop_waypoint_creation: stops the automatic random creation of waypoints
            // OR interpolate True :param plan: list of [carla.Waypoint, RoadOption] representing the route to be followed === directly through trace_route
            :param clean_queue: resets the current agent's plan
        """
        if self.interpolate:
            self._local_planner.set_global_plan(
                plan,
                stop_waypoint_creation=stop_waypoint_creation,
                clean_queue=clean_queue
            )
        else:
            self._far_planner.set_route(plan)
            self._near_planner.set_route(plan)

    def trace_route(self, start_waypoint, end_waypoint):
        """
        Calculates the shortest route between a starting and ending waypoint.

            :param start_waypoint (carla.Waypoint): initial waypoint
            :param end_waypoint (carla.Waypoint): final waypoint
        """
        start_location = start_waypoint.transform.location
        end_location = end_waypoint.transform.location
        return self._global_planner.trace_route(start_location, end_location)

    def add_emergency_stop(self):
        """
        Overwrites the throttle a brake values of a control to perform an emergency stop.
        The steering is kept the same to avoid going out of the lane when stopping during turns

            :param speed (carl.VehicleControl): control to be modified
        """
        control = carla.VehicleControl()
        control.throttle = 0.0
        control.brake = self._max_brake
        control.hand_brake = False
        return control

    def run_step(self):
        """Execute one step of navigation."""
        hazard_detected = False
        obs_actor, light_actor, walker_actor = None, None, None
        affected_by_vehicle, affected_by_change, affected_by_walker, affected_by_tlight = False, False, False, False
        hero_transform = self._vehicle.get_transform()
        speed = np.linalg.norm(carla2numpy(self._vehicle.get_velocity()))

        # Retrieve all relevant actors
        self.update_near_obs()
        vehicle_speed = get_speed(self._vehicle) / 3.6
        vehicle_loc = hero_transform.location

        target = self._near_planner.run_step(vehicle_loc)

        far_target = self._far_planner.run_step(vehicle_loc)

        if self.debug:
            self._world.debug.draw_point(target, life_time=10, color=carla.Color(255,0,0))
            self._world.debug.draw_point(far_target, life_time=10, color=carla.Color(0,255,0))

        affected_by_vehicle, obs_actor, _ = self._vehicle_obstacle_detected(self._near_object["vehicle"], target)
        if not affected_by_vehicle: # only for speed
            affected_by_change, cha_actor = self._change_lane_hazard_detected(self._near_object["vehicle"], target, far_target)
            obs_actor = cha_actor
            if not affected_by_change:
                affected_by_walker, walker_actor, _ = self._walker_obstacle_detected()

        if affected_by_vehicle or affected_by_walker or affected_by_change:
            hazard_detected = True
        else:
            # Check if the vehicle is affected by a red traffic light
            max_tlight_distance = self._base_tlight_threshold + vehicle_speed
            affected_by_tlight, light_actor = self._affected_by_traffic_light(self._near_object["tl"], max_tlight_distance)
            
            if affected_by_tlight:
                hazard_detected = True

        # 1. 跟车策略
        follow_car = False
        now_wyp = self._map.get_waypoint(hero_transform.location)
        next_lane_id = self._map.get_waypoint(self._near_planner.route[0]).lane_id
        near_lane_id = self._map.get_waypoint(target).lane_id
        now_lane_id  = now_wyp.lane_id

        # 不在换道状态, 不在junction
        if near_lane_id == (now_lane_id and next_lane_id)  and not now_wyp.is_junction \
            and len(self._near_object["car_infront"]) == 1 and abs(hero_transform.rotation.pitch)<5 \
            and not any(x is not True for x in [affected_by_walker, affected_by_change, affected_by_tlight]):
            target_vehicle = self._near_object["car_infront"][0]
            obs_wyp = self._map.get_waypoint(target_vehicle.get_location())
            if not obs_wyp.is_junction:
                obs2ego = obs_wyp.tranform.location.distanc(hero_transform.location)
                obs_vel = float(np.linalg.norm(carla2numpy(target_vehicle.get_velocity())))
                if obs2ego > max(self.STOP_THRESHOLD, 1.5*speed) and obs_vel > self.close_obs_speed_threshold*0.5:
                    target_speed = obs_vel
                    follow_car = True
                    hazard_detected = False


        if hazard_detected:
            control = self.add_emergency_stop()
        else:
            # speed_limit = self._vehicle.get_speed_limit() * self.speed_precentage  * 3.6
            control = self._vehicle_controller.run_step(self.config.max_speed, self._map.get_waypoint(target))

        return control, obs_actor, light_actor, walker_actor

    def update_near_obs(self):
        self._ego_velocity = self._vehicle.get_velocity()
        
        self._near_object = {"vehicle":[], "walker": [], "stop": [], "car_infront": [], "behind": [], "tl": []}
        actor_list = self._world.get_actors()
        vehicle_list = actor_list.filter("*vehicle*")
        
        hero_vehicle = self._vehicle
        ego_loc = hero_vehicle.get_transform().location
        pos = carla2numpy(ego_loc)
        theta = hero_vehicle.get_transform().rotation.yaw
        lights_list = self._world.get_actors().filter("*traffic_light*")
        for tl in lights_list:
            obs_loc = tl.get_location()
            if obs_loc.distance(ego_loc) < self.near_by_dis*2:
                self._near_object["tl"].append(tl)
        for v in vehicle_list:
            obs_loc = v.get_location()
            if v.id == hero_vehicle.id:
                continue
            obs_wyp = self._map.get_waypoint(obs_loc)
            p2 = carla2numpy(obs_loc)
            p3 = carla2numpy(obs_wyp.transform.location)
            obs2wy_dis = np.linalg.norm(p3 - p2)
            distance = obs_loc.distance(ego_loc)

            # 1.25m 大概是车宽
            if distance < self.near_by_dis and obs2wy_dis <= 1.25:
                angle_unnorm = self._get_angle_to(pos, theta, carla2numpy(obs_loc))
                if abs(angle_unnorm) < self.consider_angle:
                    self._near_object["vehicle"].append(v)
                elif distance < self.near_by_dis/2:
                    self._near_object["behind"].append(v)

            # update bicycle
            angle_unnorm = self._get_angle_to(pos, theta, carla2numpy(obs_loc))
            if obs_loc.distance(ego_loc) < 30 and int(v.attributes['number_of_wheels']) == 2:
                # remove obs behind us
                if abs(angle_unnorm) < 120:
                    self._near_object["walker"].append(v)

        # update walker
        for v in self._world.get_actors().filter('*walker*'):
            obs_loc = v.get_transform().location
            if obs_loc.distance(ego_loc) < 30:
                angle_unnorm = self._get_angle_to(pos, theta, carla2numpy(obs_loc))
                # remove obs behind us
                if abs(angle_unnorm) < 120:
                    self._near_object["walker"].append(v)

    def done(self):
        """Check whether the agent has reached its destination."""
        return self._local_planner.done()

    def _vehicle_obstacle_detected(self, vehicle_list, target, STOP_THRESHOLD=8):
        """
        TODO
        """
        self._ego_transform = self._vehicle.get_transform()
        o1 = _orientation(self._ego_transform.rotation.yaw)
        p1 = carla2numpy(self._ego_transform.location)
        v1 = np.linalg.norm(carla2numpy(self._ego_velocity))
        
        # 作为前向刹停的距离值
        s1 = np.clip(max(self.STOP_THRESHOLD, 2*v1), 0, self.STOP_THRESHOLD*1.5) # increases the threshold distance
        now_wyp = self._map.get_waypoint(self._ego_transform.location)
        
        for target_vehicle in vehicle_list:
            obs_wyp = self._map.get_waypoint(target_vehicle.get_location())
            o2 = _orientation(target_vehicle.get_transform().rotation.yaw)
            p2 = carla2numpy(target_vehicle.get_location())
            s2 = np.linalg.norm(carla2numpy(target_vehicle.get_velocity()))
        
            # 坐标间向量
            p2_p1 = p2 - p1
            distance = np.linalg.norm(p2_p1)
            p2_p1_hat = p2_p1 / (distance + 1e-4)

            angle_to_car = np.degrees(np.arccos(o1.dot(p2_p1_hat)))
            # to consider -ve angles too
            angle_to_car = min(angle_to_car, 360.0 - angle_to_car)
            angle_between_heading = angle2heading(o1,o2)
            ttc = TTC_Time(p1, p2, carla2numpy(self._ego_velocity), carla2numpy(target_vehicle.get_velocity()), self.too_close_dis)
            
            # -1. 如果就在即将要去的路径点上
            is_in_my_way = self._if_in_my_way(target, obs_wyp, now_wyp, near2dis = s1/2 - 2)
            if is_in_my_way:
                return (True, target_vehicle, distance)
            
            # 0. 如果不在junction内 不同road_id直接不考虑 同一road_id 不同lane_id不考虑
            if now_wyp.is_junction is False and obs_wyp.is_junction is False:
                # 速度高了还是要考虑的 -> 针对隔壁车道车突然插进来 -> 同时双方heading角度也要考虑
                if s2<7 and s2>1 and v1<7 and angle_between_heading > self.red_angle_diff * 0.9:
                    if now_wyp.road_id != obs_wyp.road_id:
                        continue
                    elif now_wyp.lane_id != obs_wyp.lane_id:
                        continue
            
            # 1. ttc 判断，对方速度接近于0时 不考虑
            if ttc < self.STOP_THRESHOLD and s2 > self.close_obs_speed_threshold:
                return (True, target_vehicle, distance)

            # 2. 跟车时刹车，距离小于刹车阈值 且车辆不在两侧范围， angle to car 适应型
            elif distance < s1 and angle_to_car < np.rad2deg(np.arcsin(now_wyp.lane_width *0.7/s1)) and angle_between_heading < self.consider_angle:
                return (True, target_vehicle, distance)

            # 3. 前方车辆heading有问题时 取消angle_to_car
            elif distance < s1 and angle_between_heading < self.consider_angle//2 and s2<2:
                return (True, target_vehicle, distance)

            # 3. junction处取消angle_to_car的限制
            elif now_wyp.is_junction and obs_wyp.is_junction:
                if distance < s1 and angle_between_heading < 180 * 0.9:
                    return (True, target_vehicle, distance)

        return (False, None, -1)

    def _if_in_my_way(self, target, obs_wyp, hero_waypoint, near2dis=2):
        """
        target: carla.Location
        """
        near_wp = self._map.get_waypoint(target)
        for i in np.linspace(0.1,near2dis,10):
            near_next_wp = near_wp.next(i)
            if len(near_next_wp)>0:
                near_next_wp = near_next_wp[0]
                wp_loc = near_next_wp.transform.location
                obs2near_dis = wp_loc.distance(obs_wyp.transform.location)
                if obs2near_dis < (hero_waypoint.lane_width*self.precentage_of_lane_staright/2):
                    return True
        return False


    def _walker_obstacle_detected(self, STOP_THRESHOLD=8):
        """
        Method to check if there is a walker in front of path or TTC
        """

        self._ego_transform = self._vehicle.get_transform()
        ego_loc = self._ego_transform.location
        theta = self._ego_transform.rotation.yaw
        pos = carla2numpy(ego_loc)

        o1 = _orientation(theta)
        p1 = carla2numpy(ego_loc)
        v1 = carla2numpy(self._ego_velocity) #km/h
        norm_v1 = round(np.linalg.norm(v1),2)
        stop_dis = np.clip(max(STOP_THRESHOLD, 2*norm_v1), 0, STOP_THRESHOLD*1.5) # increases the threshold distance
        
        for walker in self._near_object["walker"]:
            o2 = _orientation(walker.get_transform().rotation.yaw)
            v2 = carla2numpy(walker.get_velocity())
            p2 = carla2numpy(walker.get_location())
            angle_between_heading = angle2heading(o1,o2)
            dis = np.linalg.norm(p2-p1)

            obs_wyp = self._map.get_waypoint(walker.get_location())
            p2 = carla2numpy(walker.get_location())
            p3 = carla2numpy(obs_wyp.transform.location)
            obs2wy_dis = np.linalg.norm(p3 - p2)
            
            norm_v2 = round(np.linalg.norm(v2),2)
            if angle_between_heading < 10:
                continue
            if dis < stop_dis and norm_v2 > 0.3 and obs2wy_dis < max(1.6, (norm_v1+norm_v2)/2) and angle_between_heading < 120:
                return (True, walker, dis)

            # else judge ttc
            ttc_small = TTC_Time(p1, p2, v1, v2, self._map.get_waypoint(self._ego_transform.location).lane_width / 2)
            ttc_large = TTC_Time(p1, p2, v1, v2, self._map.get_waypoint(self._ego_transform.location).lane_width)
            if ttc_small < 8:
                return (True, walker, dis)
            if ttc_large <8 and angle_between_heading < 150 and np.linalg.norm(v2) > 0.5  and obs2wy_dis < 2:
                return (True, walker, dis)

        return (False, None, -1)

    def _get_angle_to(self, pos, yaw, target):
        aim = target - pos
        yaw_rad = np.deg2rad(yaw)
        # 旋转矩阵
        R = np.array([
            [np.cos(yaw_rad), -np.sin(yaw_rad)],
            [np.sin(yaw_rad),  np.cos(yaw_rad)],
            ])
        # 乘上yaw的旋转矩阵 -> 就直接在yaw角度上的坐标系了
        diff_v = R.T.dot(aim)
        # 直接就是diff_v
        angle = -np.degrees(np.arctan2(-diff_v[1], diff_v[0]))
        return angle
    
    def _change_lane_hazard_detected(self, vehicle_list, target, far_target):
        # 5m 前的远处点
        farn_lane_id = self._map.get_waypoint(far_target).lane_id
        farn_road_id = self._map.get_waypoint(far_target).road_id
        
        # 2m 前的临近点
        near_wyp = self._map.get_waypoint(target)
        # 2m 前点往回走
        near_pre_wyp = near_wyp.previous(2)[0]
        near_lane_id = near_wyp.lane_id
        near_road_id = near_wyp.road_id

        # 下一个目标点lane_id
        next_wyp = self._map.get_waypoint(self._near_planner.route[0])
        next_lane_id = next_wyp.lane_id
        next_road_id = next_wyp.road_id

        # 自身所处的lane_id
        now_wyp = self._map.get_waypoint(self._ego_transform.location)
        now_lane_id = now_wyp.lane_id
        now_road_id = now_wyp.road_id

        # 确认一下 2m目标点往回的lane id和road 和现在不一致 不然同属一个
        if (now_lane_id == near_pre_wyp.lane_id and now_road_id == near_pre_wyp.road_id) \
            or (now_lane_id == next_lane_id and now_lane_id == near_lane_id and now_lane_id == farn_lane_id):
            return (False, None)
        
        max_id = max(near_lane_id, next_lane_id, now_lane_id, farn_lane_id, near_pre_wyp.lane_id)
        min_id = min(near_lane_id, next_lane_id, now_lane_id, farn_lane_id, near_pre_wyp.lane_id)

        # 自身朝向与位置
        p1 = carla2numpy(self._vehicle.get_location())
        o1 = _orientation(self._ego_transform.rotation.yaw)
        v1 = np.linalg.norm(carla2numpy(self._ego_velocity))

        for target_vehicle in vehicle_list:

            obs_wyp = self._map.get_waypoint(target_vehicle.get_location())
            o2 = _orientation(target_vehicle.get_transform().rotation.yaw)
            p2 = carla2numpy(target_vehicle.get_location())
            v2 = np.linalg.norm(carla2numpy(target_vehicle.get_velocity()))

            # 在目标车道的车 (目标lane_id and 目标road_id)
            if (min_id <= obs_wyp.lane_id <= max_id and obs_wyp.lane_id != now_lane_id) and \
               (obs_wyp.road_id == (now_road_id or near_road_id or next_road_id or near_pre_wyp.road_id or farn_road_id)):
                angle_between_heading = angle2heading(o1,o2)
                p2 = carla2numpy(target_vehicle.get_location())
                distance = np.linalg.norm(p2-p1)
                # 远点的话 需要双重确认road_id也为远点的road id
                if (obs_wyp.lane_id == farn_lane_id and obs_wyp.road_id != farn_road_id) and distance>self.STOP_THRESHOLD:
                    continue
                # 在换道车道内的判断距离
                threshold_dis = np.clip(max(self._distance_between_change_lane, v1*2, v2*2), 0, self.STOP_THRESHOLD*1.5)
                if distance < threshold_dis and angle_between_heading < self.consider_angle * 0.65:
                    distance = np.linalg.norm(p2 - p1)
                    angle_to_car = np.degrees(np.arccos(o1.dot((p2 - p1)/ (distance + 1e-4))))
                    angle_to_car = min(angle_to_car, 360.0 - angle_to_car)
                    # 基本无速度 又不是红绿灯处停车 is_junction状态
                    if v2 < 0.1 and not near_wyp.is_junction:
                        continue
                    return (True, target_vehicle)
        return (False, None)

    def _affected_by_traffic_light(self, lights_list=None, max_distance=None):
        # -----------------------------------------------------------
        # Red light detection from SEEDs
        # -----------------------------------------------------------
        vehicle_transform = self._vehicle.get_transform()
        vehicle_location = vehicle_transform.location
        if lights_list is None:
            self._world.get_actors().filter("*traffic_light*")
        center_light_detector_bb = vehicle_transform.transform(carla.Location(x=self.center_bb_light_x, y=self.center_bb_light_y, z=self.center_bb_light_z))
        extent_light_detector_bb = carla.Vector3D(x=self.extent_bb_light_x, y=self.extent_bb_light_y, z=self.extent_bb_light_z)
        light_detector_bb = carla.BoundingBox(center_light_detector_bb, extent_light_detector_bb)
        light_detector_bb.rotation = vehicle_transform.rotation
        color2 = carla.Color(255, 255, 255, 255)

        for light in lights_list:
            # box in which we will look for traffic light triggers.            
            center_bounding_box = light.get_transform().transform(light.trigger_volume.location)
            center_bounding_box = carla.Location(center_bounding_box.x, center_bounding_box.y, center_bounding_box.z)
            length_bounding_box = carla.Vector3D(light.trigger_volume.extent.x, light.trigger_volume.extent.y, light.trigger_volume.extent.z)
            transform = carla.Transform(center_bounding_box) # can only create a bounding box from a transform.location, not from a location
            bounding_box = carla.BoundingBox(transform.location, length_bounding_box)

            gloabl_rot = light.get_transform().rotation
            bounding_box.rotation = carla.Rotation(pitch = light.trigger_volume.rotation.pitch + gloabl_rot.pitch,
                                                yaw   = light.trigger_volume.rotation.yaw   + gloabl_rot.yaw,
                                                roll  = light.trigger_volume.rotation.roll  + gloabl_rot.roll)

            if(self.check_obb_intersection(light_detector_bb, bounding_box) == True):
                if ((light.state == carla.libcarla.TrafficLightState.Red)
                    or (light.state == carla.libcarla.TrafficLightState.Yellow)):
                    return (True, light)

        return (False, None)
        
    def dot_product(self, vector1, vector2):
        return (vector1.x * vector2.x + vector1.y * vector2.y + vector1.z * vector2.z)

    def cross_product(self, vector1, vector2):
        return carla.Vector3D(x=vector1.y * vector2.z - vector1.z * vector2.y, y=vector1.z * vector2.x - vector1.x * vector2.z, z=vector1.x * vector2.y - vector1.y * vector2.x)

    def get_separating_plane(self, rPos, plane, obb1, obb2):
        ''' Checks if there is a seperating plane
        rPos Vec3
        plane Vec3
        obb1  Bounding Box
        obb2 Bounding Box
        '''
        return (abs(self.dot_product(rPos, plane)) > (abs(self.dot_product((obb1.rotation.get_forward_vector() * obb1.extent.x), plane)) +
                                                      abs(self.dot_product((obb1.rotation.get_right_vector()   * obb1.extent.y), plane)) +
                                                      abs(self.dot_product((obb1.rotation.get_up_vector()      * obb1.extent.z), plane)) +
                                                      abs(self.dot_product((obb2.rotation.get_forward_vector() * obb2.extent.x), plane)) +
                                                      abs(self.dot_product((obb2.rotation.get_right_vector()   * obb2.extent.y), plane)) +
                                                      abs(self.dot_product((obb2.rotation.get_up_vector()      * obb2.extent.z), plane)))
                )
    
    def check_obb_intersection(self, obb1, obb2):
        RPos = obb2.location - obb1.location
        return not(self.get_separating_plane(RPos, obb1.rotation.get_forward_vector(), obb1, obb2) or
                   self.get_separating_plane(RPos, obb1.rotation.get_right_vector(),   obb1, obb2) or
                   self.get_separating_plane(RPos, obb1.rotation.get_up_vector(),      obb1, obb2) or
                   self.get_separating_plane(RPos, obb2.rotation.get_forward_vector(), obb1, obb2) or
                   self.get_separating_plane(RPos, obb2.rotation.get_right_vector(),   obb1, obb2) or
                   self.get_separating_plane(RPos, obb2.rotation.get_up_vector(),      obb1, obb2) or
                   self.get_separating_plane(RPos, self.cross_product(obb1.rotation.get_forward_vector(), obb2.rotation.get_forward_vector()), obb1, obb2) or
                   self.get_separating_plane(RPos, self.cross_product(obb1.rotation.get_forward_vector(), obb2.rotation.get_right_vector()),   obb1, obb2) or
                   self.get_separating_plane(RPos, self.cross_product(obb1.rotation.get_forward_vector(), obb2.rotation.get_up_vector()),      obb1, obb2) or
                   self.get_separating_plane(RPos, self.cross_product(obb1.rotation.get_right_vector()  , obb2.rotation.get_forward_vector()), obb1, obb2) or
                   self.get_separating_plane(RPos, self.cross_product(obb1.rotation.get_right_vector()  , obb2.rotation.get_right_vector()),   obb1, obb2) or
                   self.get_separating_plane(RPos, self.cross_product(obb1.rotation.get_right_vector()  , obb2.rotation.get_up_vector()),      obb1, obb2) or
                   self.get_separating_plane(RPos, self.cross_product(obb1.rotation.get_up_vector()     , obb2.rotation.get_forward_vector()), obb1, obb2) or
                   self.get_separating_plane(RPos, self.cross_product(obb1.rotation.get_up_vector()     , obb2.rotation.get_right_vector()),   obb1, obb2) or
                   self.get_separating_plane(RPos, self.cross_product(obb1.rotation.get_up_vector()     , obb2.rotation.get_up_vector()),      obb1, obb2))


def carla2numpy(carla_vector, normalize=False):
    result = np.array([carla_vector.x, carla_vector.y])

    if normalize:
        return result / (np.linalg.norm(result) + 1e-4)

    return result

def _orientation(yaw):
    return np.float32([np.cos(np.radians(yaw)), np.sin(np.radians(yaw))])

def angle2heading(o1,o2):
    angle_between_heading = np.degrees(np.arccos(o1.dot(o2)))
    angle_between_heading = min(angle_between_heading, 360.0 - angle_between_heading)
    return angle_between_heading

def TTC_Time(pa,pb,va,vb , close_dis):
    maxt = 999
    rv = va - vb
    rp = pb - pa

    # 速度方向相反 -> vb速度va快 也算其中之一
    if rp.dot(rv) < 0.0:
        ttc = maxt
    else:
        a = np.linalg.norm(rv)
        # 速度基本一致 无需考虑
        if a <1e-4:
            return maxt
        rv_project2_rp = rp*rp.dot(rv)/rp.dot(rp)
        rp_project2_rv = rv*rv.dot(rp)/rv.dot(rv)
        dis_have_no_vel = np.linalg.norm(rp - rp_project2_rv)
        if dis_have_no_vel > close_dis:
            return maxt
        ttc = np.linalg.norm(rp)/np.linalg.norm(rv_project2_rp)
    return ttc