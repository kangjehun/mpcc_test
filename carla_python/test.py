import carla

# Connect to the client and retrieve the world object
client = carla.Client('localhost', 2000)
world = client.load_world('Town07_Opt')

# Unload all map layers except the road layer
# world.unload_map_layer(carla.MapLayer.Buildings)
# world.unload_map_layer(carla.MapLayer.Decals)
# world.unload_map_layer(carla.MapLayer.Foliage)
# world.unload_map_layer(carla.MapLayer.ParkedVehicles)
# world.unload_map_layer(carla.MapLayer.Ground)
# world.unload_map_layer(carla.MapLayer.Particles)
# world.unload_map_layer(carla.MapLayer.Foliage)
# world.unload_map_layer(carla.MapLayer.Props)
# world.unload_map_layer(carla.MapLayer.StreetLights)
# world.unload_map_layer(carla.MapLayer.Walls)
world.unload_map_layer(carla.MapLayer.All)

# Get the blueprint library and select a vehicle blueprint
blueprint_library = world.get_blueprint_library()
vehicle_bp = blueprint_library.filter('model3')[0]