from pxr import UsdGeom, UsdShade, Sdf, Gf
import omni.usd

# 0. Get the current USD stage
stage = omni.usd.get_context().get_stage()

# --- User Configurable ---
plane_path = "/World/MyImagePlane"
material_path = "/World/MyMaterial"
image_path = "/home/qpaig/Documents/fr3/fr3_sketch.jpeg"
# --------------------------

# 1. Create a Cube to act as a plane
cube = UsdGeom.Cube.Define(stage, Sdf.Path(plane_path))
prim = cube.GetPrim()

# 2. Set Scale (make it flat and wide)
if not prim.HasAttribute("xformOp:scale"):
    prim.CreateAttribute("xformOp:scale", Sdf.ValueTypeNames.Float3).Set(Gf.Vec3f(2.0, 0.01, 1.0))
    prim.CreateAttribute("xformOpOrder", Sdf.ValueTypeNames.TokenArray).Set(["xformOp:scale"])
else:
    prim.GetAttribute("xformOp:scale").Set(Gf.Vec3f(2.0, 0.01, 1.0))

# 3. Set Translate (move it up and forward)
if not prim.HasAttribute("xformOp:translate"):
    prim.CreateAttribute("xformOp:translate", Sdf.ValueTypeNames.Float3).Set(Gf.Vec3f(0.0, 0.5, 1.0))
    prim.GetAttribute("xformOpOrder").Set(["xformOp:translate", "xformOp:scale"])
else:
    prim.GetAttribute("xformOp:translate").Set(Gf.Vec3f(0.0, 0.5, 1.0))

# 4. Create Material
material = UsdShade.Material.Define(stage, Sdf.Path(material_path))

# 5. Create Shader inside the Material
shader = UsdShade.Shader.Define(stage, Sdf.Path(material_path + "/Shader"))
shader.CreateIdAttr("OmniPBR")  # Set it to OmniPBR shader

# 6. Set shader inputs (texture and fallback color)
shader.CreateInput("diffuse_texture", Sdf.ValueTypeNames.Asset).Set(image_path)
shader.CreateInput("diffuse_color", Sdf.ValueTypeNames.Float3).Set(Gf.Vec3f(1.0, 1.0, 1.0))  # white fallback color

# 7. Create shader output properly
shader_output = shader.CreateOutput("surface", Sdf.ValueTypeNames.Token)

# 8. Connect shader output to material surface
material.CreateSurfaceOutput().ConnectToSource(shader_output)

# 9. Bind the material to the cube
UsdShade.MaterialBindingAPI(prim).Bind(material)

print("✅ Successfully created textured image plane!")

