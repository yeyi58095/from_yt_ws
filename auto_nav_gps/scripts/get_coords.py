#!/usr/bin/env python3
import sys
from geopy.geocoders import Nominatim

def main():
    if len(sys.argv) < 2:
        print("Usage: rosrun auto_nav_gps get_coords.py <place_name>")
        return

    place_name = " ".join(sys.argv[1:])
    geolocator = Nominatim(user_agent="ros_geo_locator")
    location = geolocator.geocode(place_name)

    if location:
        print(f"Place: {place_name}")
        print(f"Latitude: {location.latitude}, Longitude: {location.longitude}")
    else:
        print(f"Could not find coordinates for '{place_name}'")

if __name__ == '__main__':
    main()
