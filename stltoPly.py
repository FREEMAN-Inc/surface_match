import trimesh

mesh = trimesh.load('medium.stl')
mesh.export('medium.ply')

