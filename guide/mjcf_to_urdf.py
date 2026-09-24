# Run: python3 mjcf_to_urdf.py path/to/model.xml --out robot.urdf --mesh-prefix meshes --density 1010 --base-origin body

from __future__ import annotations

import argparse
import math
import os
import xml.etree.ElementTree as ET
from xml.dom import minidom

import numpy as np
import trimesh
from scipy.spatial.transform import Rotation as R


MJF_PA12_DENSITY_KG_M3 = 1010.0


def parse_vec(text: str | None, default=None) -> np.ndarray:
    if text is None:
        return np.array([0.0, 0.0, 0.0] if default is None else default, dtype=float)
    return np.array([float(x) for x in text.split()], dtype=float)


def fmt(x: float) -> str:
    x = float(x)
    if abs(x) < 1e-15:
        return "0"
    return f"{x:.10g}"


def fmt_vec(v) -> str:
    return " ".join(fmt(float(x)) for x in v)


def deg_to_rad(x: float) -> float:
    return float(x) * math.pi / 180.0


def xyaxes_to_matrix(xyaxes: str | None) -> np.ndarray:
    if not xyaxes:
        return np.eye(3)

    vals = [float(x) for x in xyaxes.split()]
    if len(vals) != 6:
        return np.eye(3)

    x = np.array(vals[:3], dtype=float)
    y = np.array(vals[3:], dtype=float)

    x /= np.linalg.norm(x)
    y /= np.linalg.norm(y)

    z = np.cross(x, y)
    z /= np.linalg.norm(z)

    y = np.cross(z, x)
    y /= np.linalg.norm(y)

    return np.column_stack([x, y, z])


def quat_to_matrix(qtext: str | None) -> np.ndarray:
    if not qtext:
        return np.eye(3)
    q = [float(x) for x in qtext.split()]
    if len(q) != 4:
        return np.eye(3)
    return R.from_quat([q[1], q[2], q[3], q[0]]).as_matrix()


def matrix_to_rpy(M: np.ndarray) -> str:
    return fmt_vec(R.from_matrix(M).as_euler("xyz", degrees=False))


def elem_rotation(el: ET.Element, default_R: np.ndarray) -> np.ndarray:
    if "quat" in el.attrib:
        return quat_to_matrix(el.attrib.get("quat"))
    if "xyaxes" in el.attrib:
        return xyaxes_to_matrix(el.attrib.get("xyaxes"))
    return default_R


def mesh_mass_properties(mesh_path: str, scale: np.ndarray, rotation: np.ndarray, density: float):
    mesh = trimesh.load(mesh_path, force="mesh")
    if mesh.is_empty:
        raise ValueError(f"empty mesh: {mesh_path}")

    mesh.apply_scale(scale)

    T = np.eye(4)
    T[:3, :3] = rotation
    mesh.apply_transform(T)

    if not mesh.is_watertight or abs(mesh.volume) <= 1e-18:
        mesh = mesh.convex_hull

    volume = abs(float(mesh.volume))
    mass = max(volume * density, 1e-8)
    com = np.array(mesh.center_mass, dtype=float)
    inertia = np.array(mesh.moment_inertia, dtype=float) * density

    inertia = 0.5 * (inertia + inertia.T)
    inertia[np.abs(inertia) < 1e-15] = 0.0

    return mass, com, inertia


def combine_mass_properties(parts):
    total_m = sum(m for m, _, _ in parts)
    com = sum(m * c for m, c, _ in parts) / total_m

    I = np.zeros((3, 3))
    for m, c, I_com in parts:
        d = c - com
        I += I_com + m * ((np.dot(d, d) * np.eye(3)) - np.outer(d, d))

    I = 0.5 * (I + I.T)
    I[np.abs(I) < 1e-15] = 0.0
    return total_m, com, I


class Converter:
    def __init__(self, xml_path: str, out_path: str, mesh_prefix: str, density: float):
        self.xml_path = os.path.abspath(xml_path)
        self.xml_dir = os.path.dirname(self.xml_path)
        self.out_path = out_path
        self.mesh_prefix = mesh_prefix.strip("/")
        self.density = density

        self.mjcf = ET.parse(self.xml_path).getroot()
        self.robot = ET.Element("robot", {"name": self.mjcf.attrib.get("model", "converted_robot")})

        compiler = self.mjcf.find("compiler")
        self.angle_is_degree = True
        self.meshdir = self.xml_dir
        if compiler is not None:
            self.angle_is_degree = compiler.attrib.get("angle", "radian") == "degree"
            self.meshdir = os.path.join(self.xml_dir, compiler.attrib.get("meshdir", ""))

        self.default_geom_R = np.eye(3)
        self.default_rgba = "0.54 0.55 0.55 1"
        default_geom = self.find_first_default_geom(self.mjcf.find("default"))
        if default_geom is not None:
            self.default_geom_R = elem_rotation(default_geom, np.eye(3))
            self.default_rgba = default_geom.attrib.get("rgba", self.default_rgba)

        self.mesh_assets = self.parse_mesh_assets()
        self.mimics = self.parse_mimics()
        self.write_material()

    def find_first_default_geom(self, default_el):
        if default_el is None:
            return None
        g = default_el.find("geom")
        if g is not None:
            return g
        for child in default_el.findall("default"):
            g = self.find_first_default_geom(child)
            if g is not None:
                return g
        return None

    def parse_mesh_assets(self):
        assets = {}
        asset = self.mjcf.find("asset")
        if asset is None:
            return assets

        for m in asset.findall("mesh"):
            file_name = m.attrib.get("file")
            if "name" in m.attrib:
                name = m.attrib["name"]
            elif file_name:
                name = os.path.splitext(os.path.basename(file_name))[0]
            else:
                continue

            if file_name is None:
                file_name = f"{name}.stl"

            assets[name] = {
                "file": file_name,
                "path": os.path.join(self.meshdir, file_name),
                "scale": parse_vec(m.attrib.get("scale"), [1, 1, 1]),
            }
        return assets

    def parse_mimics(self):
        out = {}
        eq = self.mjcf.find("equality")
        if eq is None:
            return out

        for j in eq.findall("joint"):
            j1 = j.attrib.get("joint1")
            j2 = j.attrib.get("joint2")
            coeff = [float(x) for x in j.attrib.get("polycoef", "0 1 0 0 0").split()]
            if j1 and j2 and len(coeff) >= 2:
                out[j2] = {
                    "joint": j1,
                    "offset": deg_to_rad(coeff[0]) if self.angle_is_degree else coeff[0],
                    "multiplier": coeff[1],
                }
        return out

    def write_material(self):
        mat = ET.SubElement(self.robot, "material", {"name": "MJF_PA12"})
        ET.SubElement(mat, "color", {"rgba": self.default_rgba})

    def joint_anchor(self, body_el: ET.Element) -> np.ndarray:
        j = body_el.find("joint")
        if j is None:
            return np.zeros(3)
        return parse_vec(j.attrib.get("pos"), [0, 0, 0])

    def make_link(self, body_el: ET.Element, link_anchor: np.ndarray):
        name = body_el.attrib["name"]
        link = ET.SubElement(self.robot, "link", {"name": name})
        mass_parts = []

        for geom in body_el.findall("geom"):
            mesh_name = geom.attrib.get("mesh")
            if not mesh_name or mesh_name not in self.mesh_assets:
                continue

            asset = self.mesh_assets[mesh_name]
            mesh_file = asset["file"]
            mesh_path = asset["path"]
            mesh_scale = asset["scale"]

            geom_pos = parse_vec(geom.attrib.get("pos"), [0, 0, 0])
            geom_R = elem_rotation(geom, self.default_geom_R)

            visual_xyz = geom_pos - link_anchor
            visual_rpy = matrix_to_rpy(geom_R)
            filename = f"{self.mesh_prefix}/{mesh_file}" if self.mesh_prefix else mesh_file

            for tag in ("visual", "collision"):
                block = ET.SubElement(link, tag)
                ET.SubElement(block, "origin", {"xyz": fmt_vec(visual_xyz), "rpy": visual_rpy})
                geo = ET.SubElement(block, "geometry")
                ET.SubElement(geo, "mesh", {"filename": filename, "scale": fmt_vec(mesh_scale)})
                if tag == "visual":
                    ET.SubElement(block, "material", {"name": "MJF_PA12"})

            if os.path.exists(mesh_path):
                try:
                    m, c, I = mesh_mass_properties(mesh_path, mesh_scale, geom_R, self.density)
                    c = c + geom_pos - link_anchor
                    mass_parts.append((m, c, I))
                except Exception as e:
                    print(f"WARNING: failed inertia for {name}/{mesh_file}: {e}")
            else:
                print(f"WARNING: missing mesh for inertia: {mesh_path}")

        if mass_parts:
            mass, com, inertia = combine_mass_properties(mass_parts)
        else:
            mass = 1e-4
            com = np.zeros(3)
            inertia = np.eye(3) * 1e-8

        inertial = ET.Element("inertial")
        ET.SubElement(inertial, "origin", {"xyz": fmt_vec(com), "rpy": "0 0 0"})
        ET.SubElement(inertial, "mass", {"value": fmt(mass)})
        ET.SubElement(inertial, "inertia", {
            "ixx": fmt(inertia[0, 0]),
            "ixy": fmt(inertia[0, 1]),
            "ixz": fmt(inertia[0, 2]),
            "iyy": fmt(inertia[1, 1]),
            "iyz": fmt(inertia[1, 2]),
            "izz": fmt(inertia[2, 2]),
        })
        link.insert(0, inertial)

    def make_joint(self, parent_name: str, child_name: str, child_body_el: ET.Element, parent_anchor: np.ndarray):
        child_joint_el = child_body_el.find("joint")
        if child_joint_el is None:
            return

        name = child_joint_el.attrib["name"]
        body_pos = parse_vec(child_body_el.attrib.get("pos"), [0, 0, 0])
        body_R = quat_to_matrix(child_body_el.attrib.get("quat"))
        child_anchor = parse_vec(child_joint_el.attrib.get("pos"), [0, 0, 0])

        joint_xyz = body_pos + body_R @ child_anchor - parent_anchor
        joint_rpy = matrix_to_rpy(body_R)
        axis = parse_vec(child_joint_el.attrib.get("axis"), [0, 0, 1])

        joint = ET.SubElement(self.robot, "joint", {"name": name, "type": "revolute"})
        ET.SubElement(joint, "parent", {"link": parent_name})
        ET.SubElement(joint, "child", {"link": child_name})
        ET.SubElement(joint, "origin", {"xyz": fmt_vec(joint_xyz), "rpy": joint_rpy})
        ET.SubElement(joint, "axis", {"xyz": fmt_vec(axis)})

        rng = child_joint_el.attrib.get("range")
        if rng:
            lo, hi = [float(x) for x in rng.split()]
            if self.angle_is_degree:
                lo, hi = deg_to_rad(lo), deg_to_rad(hi)
        else:
            lo, hi = -math.pi, math.pi

        ET.SubElement(joint, "limit", {
            "lower": fmt(lo),
            "upper": fmt(hi),
            "effort": "10",
            "velocity": "10",
        })

        if name in self.mimics:
            m = self.mimics[name]
            ET.SubElement(joint, "mimic", {
                "joint": m["joint"],
                "multiplier": fmt(m["multiplier"]),
                "offset": fmt(m["offset"]),
            })

    def body_mesh_center(self, body_el: ET.Element) -> np.ndarray:
        vertices = []
        for geom in body_el.findall("geom"):
            mesh_name = geom.attrib.get("mesh")
            if not mesh_name or mesh_name not in self.mesh_assets:
                continue

            asset = self.mesh_assets[mesh_name]
            mesh_path = asset["path"]
            if not os.path.exists(mesh_path):
                continue

            mesh = trimesh.load(mesh_path, force="mesh")
            if mesh.is_empty:
                continue

            mesh.apply_scale(asset["scale"])
            geom_R = elem_rotation(geom, self.default_geom_R)
            geom_pos = parse_vec(geom.attrib.get("pos"), [0, 0, 0])
            v = (geom_R @ np.asarray(mesh.vertices).T).T + geom_pos
            vertices.append(v)

        if not vertices:
            return np.zeros(3)

        v_all = np.vstack(vertices)
        return 0.5 * (v_all.min(axis=0) + v_all.max(axis=0))

    def body_mesh_com(self, body_el: ET.Element) -> np.ndarray:
        parts = []
        for geom in body_el.findall("geom"):
            mesh_name = geom.attrib.get("mesh")
            if not mesh_name or mesh_name not in self.mesh_assets:
                continue

            asset = self.mesh_assets[mesh_name]
            mesh_path = asset["path"]
            if not os.path.exists(mesh_path):
                continue

            geom_R = elem_rotation(geom, self.default_geom_R)
            geom_pos = parse_vec(geom.attrib.get("pos"), [0, 0, 0])

            try:
                m, c, _ = mesh_mass_properties(mesh_path, asset["scale"], geom_R, self.density)
                parts.append((m, c + geom_pos))
            except Exception:
                pass

        if not parts:
            return np.zeros(3)

        total = sum(m for m, _ in parts)
        return sum(m * c for m, c in parts) / total


    def has_mesh_geom(self, body_el: ET.Element) -> bool:
        for geom in body_el.findall("geom"):
            mesh_name = geom.attrib.get("mesh")
            if mesh_name and mesh_name in self.mesh_assets:
                return True
        return False

    def body_local_transform(self, body_el: ET.Element):
        pos = parse_vec(body_el.attrib.get("pos"), [0, 0, 0])
        if "quat" in body_el.attrib:
            rot = quat_to_matrix(body_el.attrib.get("quat"))
        elif "euler" in body_el.attrib:
            e = parse_vec(body_el.attrib.get("euler"), [0, 0, 0])
            rot = R.from_euler("xyz", e, degrees=False).as_matrix()
        else:
            rot = np.eye(3)
        return pos, rot

    def make_fixed_joint(self, parent_name: str, child_name: str, joint_xyz: np.ndarray, joint_R: np.ndarray):
        safe_child = child_name.replace("/", "_").strip("_") or "body"
        joint = ET.SubElement(self.robot, "joint", {"name": f"fixed_{parent_name.replace('/', '_')}_to_{safe_child}", "type": "fixed"})
        ET.SubElement(joint, "parent", {"link": parent_name})
        ET.SubElement(joint, "child", {"link": child_name})
        ET.SubElement(joint, "origin", {"xyz": fmt_vec(joint_xyz), "rpy": matrix_to_rpy(joint_R)})

    def make_joint_from_pose(self, parent_name: str, child_name: str, child_body_el: ET.Element,
                             parent_anchor: np.ndarray, body_pos: np.ndarray, body_R: np.ndarray):
        child_joint_el = child_body_el.find("joint")
        if child_joint_el is None:
            return

        name = child_joint_el.attrib["name"]
        child_anchor = parse_vec(child_joint_el.attrib.get("pos"), [0, 0, 0])

        joint_xyz = body_pos + body_R @ child_anchor - parent_anchor
        joint_rpy = matrix_to_rpy(body_R)
        axis = parse_vec(child_joint_el.attrib.get("axis"), [0, 0, 1])

        joint = ET.SubElement(self.robot, "joint", {"name": name, "type": "revolute"})
        ET.SubElement(joint, "parent", {"link": parent_name})
        ET.SubElement(joint, "child", {"link": child_name})
        ET.SubElement(joint, "origin", {"xyz": fmt_vec(joint_xyz), "rpy": joint_rpy})
        ET.SubElement(joint, "axis", {"xyz": fmt_vec(axis)})

        rng = child_joint_el.attrib.get("range")
        if rng:
            lo, hi = [float(x) for x in rng.split()]
            if self.angle_is_degree:
                lo, hi = deg_to_rad(lo), deg_to_rad(hi)
        else:
            lo, hi = -math.pi, math.pi

        ET.SubElement(joint, "limit", {
            "lower": fmt(lo),
            "upper": fmt(hi),
            "effort": "10",
            "velocity": "10",
        })

        if name in self.mimics:
            m = self.mimics[name]
            ET.SubElement(joint, "mimic", {
                "joint": m["joint"],
                "multiplier": fmt(m["multiplier"]),
                "offset": fmt(m["offset"]),
            })

    def recurse_body(self, body_el: ET.Element, parent_name: str, parent_anchor: np.ndarray, carry_pos=None, carry_R=None):
        if carry_pos is None:
            carry_pos = np.zeros(3)
        if carry_R is None:
            carry_R = np.eye(3)

        local_pos, local_R = self.body_local_transform(body_el)
        body_pos = carry_pos + carry_R @ local_pos
        body_R = carry_R @ local_R

        has_joint = body_el.find("joint") is not None
        has_geom = self.has_mesh_geom(body_el)

        if not has_joint and not has_geom:
            for child in body_el.findall("body"):
                self.recurse_body(child, parent_name, parent_anchor, body_pos, body_R)
            return

        name = body_el.attrib["name"]

        if has_joint:
            this_anchor = self.joint_anchor(body_el)
        elif parent_name == "mjcf_world" and getattr(self, "base_origin", "body") == "mesh_center":
            this_anchor = self.body_mesh_center(body_el)
        elif parent_name == "mjcf_world" and getattr(self, "base_origin", "body") == "com":
            this_anchor = self.body_mesh_com(body_el)
        else:
            this_anchor = np.zeros(3)

        self.make_link(body_el, this_anchor)

        if has_joint:
            self.make_joint_from_pose(parent_name, name, body_el, parent_anchor, body_pos, body_R)
        else:
            self.make_fixed_joint(parent_name, name, body_pos - parent_anchor, body_R)

        for child in body_el.findall("body"):
            self.recurse_body(child, name, this_anchor, np.zeros(3), np.eye(3))

    def convert(self):
        world = self.mjcf.find("worldbody")
        if world is None:
            raise ValueError("No <worldbody> in MJCF")

        ET.SubElement(self.robot, "link", {"name": "mjcf_world"})
        for body in world.findall("body"):
            self.recurse_body(body, "mjcf_world", np.zeros(3), np.zeros(3), np.eye(3))

    def write(self):
        raw = ET.tostring(self.robot, encoding="utf-8")
        pretty = minidom.parseString(raw).toprettyxml(indent="  ")
        pretty = "\n".join(line for line in pretty.splitlines() if line.strip())
        with open(self.out_path, "w", encoding="utf-8") as f:
            f.write(pretty + "\n")


def main():
    p = argparse.ArgumentParser()
    p.add_argument("xml")
    p.add_argument("--out", default="robot.urdf")
    p.add_argument("--mesh-prefix", default="meshes")
    p.add_argument("--density", type=float, default=MJF_PA12_DENSITY_KG_M3)
    p.add_argument(
        "--base-origin",
        choices=["body", "mesh_center", "com"],
        default="body",
        help="Root/base link origin. Use body for MJCF models with articulated body pos/quat transforms.",
    )
    args = p.parse_args()

    c = Converter(args.xml, args.out, args.mesh_prefix, args.density)
    c.base_origin = args.base_origin
    c.convert()
    c.write()

    print(f"Wrote {args.out}")
    print(f"MJF_PA12 density: {args.density} kg/m^3")
    print(f"Base origin mode: {args.base_origin}")


if __name__ == "__main__":
    main()
