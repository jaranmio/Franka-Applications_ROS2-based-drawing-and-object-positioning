from pxr import Usd, UsdGeom, Gf
import omni.usd
import omni.kit.commands

stage = omni.usd.get_context().get_stage()

# === Parameters
tip_radius = 0.002
tip_offset_z = 0.015
tcp_path = "/World/fr3/fr3_hand_tcp"
tip_path = tcp_path + "/pencil_tip"  # 🔁 Nested under TCP

# === 1. Create tip under TCP
if not stage.GetPrimAtPath(tip_path):
    omni.kit.commands.execute(
        "CreatePrim", prim_type="Sphere", prim_path=tip_path,
        attributes={"radius": tip_radius}
    )

# === 2. Apply local Z offset relative to TCP
tip_prim = stage.GetPrimAtPath(tip_path)
tip_xform = UsdGeom.Xformable(tip_prim)

# Apply local transform (relative to TCP)
existing_translate = [op for op in tip_xform.GetOrderedXformOps() if op.GetOpType() == UsdGeom.XformOp.TypeTranslate]
if existing_translate:
    existing_translate[0].Set(Gf.Vec3d(0, 0, tip_offset_z))
else:
    tip_xform.AddTranslateOp().Set(Gf.Vec3d(0, 0, tip_offset_z))

