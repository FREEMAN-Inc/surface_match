import trimesh

# 載入 STL
mesh = trimesh.load("2-3.STL")


# 將頂點從 mm → m
mesh.vertices *= 0.001

# 儲存為新 STL
mesh.export("2-3m.stl")
print("Converted model saved to 2-3 m.stl")

