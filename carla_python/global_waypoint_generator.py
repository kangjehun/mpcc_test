import carla
from agents.navigation.global_route_planner import GlobalRoutePlanner

if __name__ == '__main__':
    client = carla.Client("localhost", 2000)
    client.set_timeout(10)
    world = client.load_world('Town07_Opt')
    world.unload_map_layer(carla.MapLayer.All)
    amap = world.get_map()

    # create global route planner
    sampling_resolution = 2
    grp = GlobalRoutePlanner(amap, sampling_resolution)
    # Get available spawn points in the map
    spawn_points = world.get_map().get_spawn_points()

    for i, sp in enumerate(spawn_points):
        print(f"Spawn Point {i}: {sp.location}")
        # Draw Index on the World
        world.debug.draw_string(sp.location, str(i), draw_shadow=False, color=carla.Color(255, 255, 255), life_time=1000, persistent_lines=True)

    # Draw a path passing through multiple points
    points = [spawn_points[32].location, spawn_points[57].location, spawn_points[97].location,
              spawn_points[10].location, spawn_points[19].location, spawn_points[39].location,
              spawn_points[32].location]
    wpts = []
    for i in range(len(points)-1):
        w1 = grp.trace_route(points[i], points[i+1])
        wpts += w1
        for w in w1:
            # Denote the all the created points on the map
            world.debug.draw_string(w[0].transform.location, 'O', draw_shadow=False, color=carla.Color(r=0, g=0, b=255), life_time=1000.0, persistent_lines=True)
    # Save the x, y, and z coordinates of each waypoint and its index to a file
    with open('waypoints.txt', 'w') as f:
        for i, w in enumerate(wpts):
            f.write(f"Waypoint {i}: {w[0].transform.location.x}, {w[0].transform.location.y}, {w[0].transform.location.z}\n")

    # # Get the blueprint library and select a vehicle blueprint
    # blueprint_library = world.get_blueprint_library()
    # vehicle_bp = blueprint_library.filter('model3')[0]