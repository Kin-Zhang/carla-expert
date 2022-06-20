#!/usr/bin/env python
# Copyright (c) 2018-2019 Intel Labs.
# authors: German Ros (german.ros@intel.com), Felipe Codevilla (felipe.alcm@gmail.com)
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

"""
Module to manipulate the routes, by making then more or less dense (Up to a certain parameter).
It also contains functions to convert the CARLA world location do GPS coordinates.
"""

import math
import xml.etree.ElementTree as ET
import carla
import numpy as np

from .map_utils import RoadOption

EARTH_RADIUS_EQUA = 6378137.0


def location_to_gps(location):

    lon = location.x * 180.0 / (math.pi * EARTH_RADIUS_EQUA)
    lat = 360.0 * math.atan(math.exp(-location.y / EARTH_RADIUS_EQUA)) / math.pi - 90.0
    z = location.z

    return (lat, lon, z)


def gps_to_location(gps):
    lat, lon, z = gps
    lat = float(lat)
    lon = float(lon)
    z = float(z)

    location = carla.Location(z=z)

    location.x = lon / 180.0 * (math.pi * EARTH_RADIUS_EQUA)

    location.y = -1.0 * math.log(math.tan((lat + 90.0) * math.pi / 360.0)) * EARTH_RADIUS_EQUA

    return location


def _location_to_gps_leaderbaord(lat_ref, lon_ref, location):
    """
    Convert from world coordinates to GPS coordinates
    :param lat_ref: latitude reference for the current map
    :param lon_ref: longitude reference for the current map
    :param location: location to translate
    :return: dictionary with lat, lon and height
    """

    EARTH_RADIUS_EQUA = 6378137.0   # pylint: disable=invalid-name
    scale = math.cos(lat_ref * math.pi / 180.0)
    mx = scale * lon_ref * math.pi * EARTH_RADIUS_EQUA / 180.0
    my = scale * EARTH_RADIUS_EQUA * math.log(math.tan((90.0 + lat_ref) * math.pi / 360.0))
    mx += location.x
    my -= location.y

    lon = mx * 180.0 / (math.pi * EARTH_RADIUS_EQUA * scale)
    lat = 360.0 * math.atan(math.exp(my / (EARTH_RADIUS_EQUA * scale))) / math.pi - 90.0
    z = location.z

    return {'lat': lat, 'lon': lon, 'z': z}


def location_route_to_gps(route):
    """
        Locate each waypoint of the route into gps, (lat long ) representations.
    :param route:
    :param lat_ref:
    :param lon_ref:
    :return:
    """
    # lat_ref, lon_ref = _get_latlon_ref(world)

    gps_route = []

    for wp, connection in route:
        gps_point = location_to_gps(wp.transform.location)
        gps_route.append((gps_point, connection))

    return gps_route


def _get_latlon_ref(world):
    """
    Convert from waypoints world coordinates to CARLA GPS coordinates
    :return: tuple with lat and lon coordinates
    """
    xodr = world.get_map().to_opendrive()
    tree = ET.ElementTree(ET.fromstring(xodr))

    # default reference
    lat_ref = 42.0
    lon_ref = 2.0

    for opendrive in tree.iter("OpenDRIVE"):
        for header in opendrive.iter("header"):
            for georef in header.iter("geoReference"):
                if georef.text:
                    str_list = georef.text.split(' ')
                    for item in str_list:
                        if '+lat_0' in item:
                            lat_ref = float(item.split('=')[1])
                        if '+lon_0' in item:
                            lon_ref = float(item.split('=')[1])
    return lat_ref, lon_ref


def downsample_route(route, sample_factor):
    """
    Downsample the route by some factor.
    :param route: the trajectory , has to contain the waypoints and the road options
    :param sample_factor: Maximum distance between samples
    :return: returns the ids of the final route that can
    """

    ids_to_sample = []
    prev_option = None
    dist = 0

    for i, point in enumerate(route):
        curr_option = point[1]

        # Lane changing
        if curr_option in (RoadOption.CHANGELANELEFT, RoadOption.CHANGELANERIGHT):
            ids_to_sample.append(i)
            dist = 0

        # When road option changes
        elif prev_option != curr_option and prev_option not in (RoadOption.CHANGELANELEFT, RoadOption.CHANGELANERIGHT):
            ids_to_sample.append(i)
            dist = 0

        # After a certain max distance
        elif dist > sample_factor:
            ids_to_sample.append(i)
            dist = 0

        # At the end
        elif i == len(route) - 1:
            ids_to_sample.append(i)
            dist = 0

        # Compute the distance traveled
        else:
            curr_location = point[0].transform.location
            prev_location = route[i-1][0].transform.location
            dist += curr_location.distance(prev_location)

        prev_option = curr_option

    return ids_to_sample
