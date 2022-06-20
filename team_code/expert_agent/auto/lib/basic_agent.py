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
from .misc import get_speed, is_within_distance, get_trafficlight_trigger_location, compute_distance
import numpy as np

class BasicAgent(object):
    """
    BasicAgent implements an agent that navigates the scene.
    This agent respects traffic lights and other vehicles, but ignores stop signs.
    It has several functions available to specify the route that the agent must follow,
    as well as to change its parameters in case a different driving mode is desired.
    """

    def __init__(self, vehicle, target_speed=20, opt_dict={}):
        """
        Initialization the agent paramters, the local and the global planner.

            :param vehicle: actor to apply to agent logic onto
            :param target_speed: speed (in Km/h) at which the vehicle will move
            :param opt_dict: dictionary in case some of its parameters want to be changed.
                This also applies to parameters related to the LocalPlanner.
        """
        self._vehicle = vehicle
        self._world = self._vehicle.get_world()
        self._map = self._world.get_map()
        self._last_traffic_light = None

        # Base parameters
        self._ignore_traffic_lights = False
        self._ignore_stop_signs = False
        self._ignore_vehicles = False
        self._target_speed = target_speed
        self._sampling_resolution = 2.0
        self._base_tlight_threshold = 5.0  # meters
        self._base_vehicle_threshold = 5.0  # meters
        self._max_brake = 0.5
        self.STOP_THRESHOLD = 8

        # Change parameters according to the dictionary
        opt_dict['target_speed'] = target_speed
        if 'ignore_traffic_lights' in opt_dict:
            self._ignore_traffic_lights = opt_dict['ignore_traffic_lights']
        if 'ignore_stop_signs' in opt_dict:
            self._ignore_stop_signs = opt_dict['ignore_stop_signs']
        if 'ignore_vehicles' in opt_dict:
            self._ignore_vehicles = opt_dict['ignore_vehicles']
        if 'sampling_resolution' in opt_dict:
            self._sampling_resolution = opt_dict['sampling_resolution']
        if 'base_tlight_threshold' in opt_dict:
            self._base_tlight_threshold = opt_dict['base_tlight_threshold']
        if 'base_vehicle_threshold' in opt_dict:
            self._base_vehicle_threshold = opt_dict['base_vehicle_threshold']
        if 'max_brake' in opt_dict:
            self._max_steering = opt_dict['max_brake']

        # Initialize the planners
        self._local_planner = LocalPlanner(self._vehicle, opt_dict=opt_dict)
        self._global_planner = GlobalRoutePlanner(self._map, self._sampling_resolution)
        
        # Red light detection from SEED
        # Coordinates of the center of the red light detector bounding box. In local coordinates of the vehicle, units are meters
        self.center_bb_light_x = -2.0
        self.center_bb_light_y = 0.0
        self.center_bb_light_z = 0.0

        # Extent of the red light detector bounding box. In local coordinates of the vehicle, units are meters. Size are half of the bounding box
        self.extent_bb_light_x = 4.5
        self.extent_bb_light_y = 1.5
        self.extent_bb_light_z = 2.0

    def add_emergency_stop(self, control):
        """
        Overwrites the throttle a brake values of a control to perform an emergency stop.
        The steering is kept the same to avoid going out of the lane when stopping during turns

            :param speed (carl.VehicleControl): control to be modified
        """
        control.throttle = 0.0
        control.brake = self._max_brake
        control.hand_brake = False
        return control

    def set_target_speed(self, speed):
        """
        Changes the target speed of the agent
            :param speed (float): target speed in Km/h
        """
        self._local_planner.set_speed(speed)

    def follow_speed_limits(self, value=True):
        """
        If active, the agent will dynamically change the target speed according to the speed limits

            :param value (bool): whether or not to activate this behavior
        """
        self._local_planner.follow_speed_limits(value)

    def get_local_planner(self):
        """Get method for protected member local planner"""
        return self._local_planner

    def get_global_planner(self):
        """Get method for protected member local planner"""
        return self._global_planner

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

            :param plan: list of [carla.Waypoint, RoadOption] representing the route to be followed
            :param stop_waypoint_creation: stops the automatic random creation of waypoints
            :param clean_queue: resets the current agent's plan
        """
        self._local_planner.set_global_plan(
            plan,
            stop_waypoint_creation=stop_waypoint_creation,
            clean_queue=clean_queue
        )

    def trace_route(self, start_waypoint, end_waypoint):
        """
        Calculates the shortest route between a starting and ending waypoint.

            :param start_waypoint (carla.Waypoint): initial waypoint
            :param end_waypoint (carla.Waypoint): final waypoint
        """
        start_location = start_waypoint.transform.location
        end_location = end_waypoint.transform.location
        return self._global_planner.trace_route(start_location, end_location)

    def run_step(self):
        """Execute one step of navigation."""
        hazard_detected = False

        # Retrieve all relevant actors
        actor_list = self._world.get_actors()
        vehicle_list = actor_list.filter("*vehicle*")
        lights_list = actor_list.filter("*traffic_light*")
        light_actor = None
        vehicle_speed = get_speed(self._vehicle) / 3.6

        # Check for possible vehicle obstacles
        max_vehicle_distance = self._base_vehicle_threshold + vehicle_speed
        affected_by_vehicle, obs_actor, _ = self._vehicle_obstacle_detected(vehicle_list, max_vehicle_distance)
        affected_by_walker, walker_actor, _ = self._walker_obstacle_detected()


        if affected_by_vehicle or affected_by_walker:
            hazard_detected = True
        else:
            # Check if the vehicle is affected by a red traffic light
            max_tlight_distance = self._base_tlight_threshold + vehicle_speed
            affected_by_tlight, light_actor = self._affected_by_traffic_light(lights_list, max_tlight_distance)
            
            if affected_by_tlight:
                hazard_detected = True

        control = self._local_planner.run_step()
        if hazard_detected:
            control = self.add_emergency_stop(control)

        return control, obs_actor, light_actor, walker_actor

    def done(self):
        """Check whether the agent has reached its destination."""
        return self._local_planner.done()

    def ignore_traffic_lights(self, active=True):
        """(De)activates the checks for traffic lights"""
        self._ignore_traffic_lights = active

    def ignore_stop_signs(self, active=True):
        """(De)activates the checks for stop signs"""
        self._ignore_stop_signs = active

    def ignore_vehicles(self, active=True):
        """(De)activates the checks for stop signs"""
        self._ignore_vehicles = active

    def _vehicle_obstacle_detected(self, vehicle_list=None, max_distance=None, up_angle_th=90, low_angle_th=0, lane_offset=0):
        """
        Method to check if there is a vehicle in front of the agent blocking its path.

            :param vehicle_list (list of carla.Vehicle): list contatining vehicle objects.
                If None, all vehicle in the scene are used
            :param max_distance: max freespace to check for obstacles.
                If None, the base threshold value is used
        """
        if self._ignore_vehicles:
            return (False, None, -1)

        if not vehicle_list:
            vehicle_list = self._world.get_actors().filter("*vehicle*")

        if not max_distance:
            max_distance = self._base_vehicle_threshold

        ego_transform = self._vehicle.get_transform()
        ego_wpt = self._map.get_waypoint(self._vehicle.get_location())

        # Get the right offset
        if ego_wpt.lane_id < 0 and lane_offset != 0:
            lane_offset *= -1

        # Get the transform of the front of the ego
        ego_forward_vector = ego_transform.get_forward_vector()
        ego_extent = self._vehicle.bounding_box.extent.x
        ego_front_transform = ego_transform
        ego_front_transform.location += carla.Location(
            x=ego_extent * ego_forward_vector.x,
            y=ego_extent * ego_forward_vector.y,
        )

        for target_vehicle in vehicle_list:
            target_transform = target_vehicle.get_transform()
            target_wpt = self._map.get_waypoint(target_transform.location, lane_type=carla.LaneType.Any)

            # Simplified version for outside junctions
            if not ego_wpt.is_junction or not target_wpt.is_junction:

                if target_wpt.road_id != ego_wpt.road_id or target_wpt.lane_id != ego_wpt.lane_id  + lane_offset:
                    next_wpt = self._local_planner.get_incoming_waypoint_and_direction(steps=3)[0]
                    if not next_wpt:
                        continue
                    if target_wpt.road_id != next_wpt.road_id or target_wpt.lane_id != next_wpt.lane_id  + lane_offset:
                        continue

                target_forward_vector = target_transform.get_forward_vector()
                target_extent = target_vehicle.bounding_box.extent.x
                target_rear_transform = target_transform
                target_rear_transform.location -= carla.Location(
                    x=target_extent * target_forward_vector.x,
                    y=target_extent * target_forward_vector.y,
                )

                if is_within_distance(target_rear_transform, ego_front_transform, max_distance, [low_angle_th, up_angle_th]):
                    return (True, target_vehicle, compute_distance(target_transform.location, ego_transform.location))

            # Waypoints aren't reliable, check the proximity of the vehicle to the route
            else:
                route_bb = []
                ego_location = ego_transform.location
                extent_y = self._vehicle.bounding_box.extent.y
                r_vec = ego_transform.get_right_vector()
                p1 = ego_location + carla.Location(extent_y * r_vec.x, extent_y * r_vec.y)
                p2 = ego_location + carla.Location(-extent_y * r_vec.x, -extent_y * r_vec.y)
                route_bb.append([p1.x, p1.y, p1.z])
                route_bb.append([p2.x, p2.y, p2.z])

                for wp, _ in self._local_planner.get_plan():
                    if ego_location.distance(wp.transform.location) > max_distance:
                        break

                    r_vec = wp.transform.get_right_vector()
                    p1 = wp.transform.location + carla.Location(extent_y * r_vec.x, extent_y * r_vec.y)
                    p2 = wp.transform.location + carla.Location(-extent_y * r_vec.x, -extent_y * r_vec.y)
                    route_bb.append([p1.x, p1.y, p1.z])
                    route_bb.append([p2.x, p2.y, p2.z])

                if len(route_bb) < 3:
                    # 2 points don't create a polygon, nothing to check
                    return (False, None, -1)
                ego_polygon = Polygon(route_bb)

                # Compare the two polygons
                for target_vehicle in vehicle_list:
                    target_extent = target_vehicle.bounding_box.extent.x
                    if target_vehicle.id == self._vehicle.id:
                        continue
                    if ego_location.distance(target_vehicle.get_location()) > max_distance:
                        continue

                    target_bb = target_vehicle.bounding_box
                    target_vertices = target_bb.get_world_vertices(target_vehicle.get_transform())
                    target_list = [[v.x, v.y, v.z] for v in target_vertices]
                    target_polygon = Polygon(target_list)

                    if ego_polygon.intersects(target_polygon):
                        return (True, target_vehicle, compute_distance(target_vehicle.get_location(), ego_location))

                return (False, None, -1)

        return (False, None, -1)

    def _walker_obstacle_detected(self, STOP_THRESHOLD=8):
        """
        Method to check if there is a vehicle in front of the agent blocking its path.

            :param walker_list (list of carla.Actor): list contatining walker objects including bicycle.
                If None, all vehicle in the scene are used
            :param max_distance: max freespace to check for obstacles.
                If None, the base threshold value is used
        """
        if self._ignore_vehicles:
            return (False, None, -1)

        self._ego_transform = self._vehicle.get_transform()
        ego_loc = self._ego_transform.location
        theta = self._ego_transform.rotation.yaw
        pos = carla2numpy(ego_loc)
        walkers_list = []

        # update bicycle
        for v in self._world.get_actors().filter("*vehicle*"):
            obs_loc = v.get_transform().location
            angle_unnorm = self._get_angle_to(pos, theta, carla2numpy(obs_loc))

            if obs_loc.distance(ego_loc) < 30 and int(v.attributes['number_of_wheels']) == 2:
                # remove obs behind us
                if abs(angle_unnorm) < 120:
                    walkers_list.append(v)

        # update walker
        for v in self._world.get_actors().filter('*walker*'):
            obs_loc = v.get_transform().location
            if obs_loc.distance(ego_loc) < 30:
                angle_unnorm = self._get_angle_to(pos, theta, carla2numpy(obs_loc))
                # remove obs behind us
                if abs(angle_unnorm) < 120:
                    walkers_list.append(v)

        o1 = _orientation(theta)
        p1 = carla2numpy(ego_loc)
        v1 = carla2numpy(self._vehicle.get_velocity()) #km/h
        norm_v1 = round(np.linalg.norm(v1),2)
        stop_dis = np.clip(max(STOP_THRESHOLD, 2*norm_v1), 0, STOP_THRESHOLD*1.5) # increases the threshold distance
        
        for walker in walkers_list:
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