client = RemoteAPIClient();
sim = client.require('sim');

sim.startSimulation();

handle = sim.getObject("/Franka");
shapehandles = sim.getObjectsInTree(handle, sim.object_shape_type, 0);


sim.stopSimulation();