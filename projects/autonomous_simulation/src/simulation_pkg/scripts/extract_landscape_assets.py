#!/usr/bin/env python3
"""Extract selected objects from Blender OBJ packs for Gazebo use."""

from pathlib import Path


def extract(source, destination, selected_objects, mtllib, target_height, material_map):
    vertices = []
    faces = []
    current_object = None
    current_material = "default"

    for raw in Path(source).read_text(encoding="utf-8", errors="replace").splitlines():
        parts = raw.split()
        if not parts:
            continue
        if parts[0] == "o":
            current_object = " ".join(parts[1:])
        elif parts[0] == "usemtl":
            current_material = material_map.get(parts[1], parts[1])
        elif parts[0] == "v":
            vertices.append(tuple(map(float, parts[1:4])))
        elif parts[0] == "f" and current_object in selected_objects:
            indices = [int(token.split("/")[0]) for token in parts[1:]]
            faces.append((current_material, indices))

    used = sorted({index for _, face in faces for index in face})
    if not used:
        raise RuntimeError(f"No faces selected from {source}")

    chosen = [vertices[index - 1] for index in used]
    # Blender OBJ is Y-up here. Convert to Gazebo Z-up: (x, y, z) -> (x, -z, y).
    converted = [(x, -z, y) for x, y, z in chosen]
    min_x = min(v[0] for v in converted)
    max_x = max(v[0] for v in converted)
    min_y = min(v[1] for v in converted)
    max_y = max(v[1] for v in converted)
    min_z = min(v[2] for v in converted)
    max_z = max(v[2] for v in converted)
    scale = target_height / (max_z - min_z)
    center_x = (min_x + max_x) / 2.0
    center_y = (min_y + max_y) / 2.0

    normalized = [
        ((x - center_x) * scale, (y - center_y) * scale, (z - min_z) * scale)
        for x, y, z in converted
    ]
    remap = {old: new for new, old in enumerate(used, start=1)}

    lines = [
        "# Extracted and normalized for Gazebo from a local Blender OBJ pack.",
        f"mtllib {mtllib}",
        "o landscape_asset",
        "s off",
    ]
    lines.extend(f"v {x:.6f} {y:.6f} {z:.6f}" for x, y, z in normalized)
    last_material = None
    for material, face in faces:
        if material != last_material:
            lines.append(f"usemtl {material}")
            last_material = material
        lines.append("f " + " ".join(str(remap[index]) for index in face))
    Path(destination).write_text("\n".join(lines) + "\n", encoding="utf-8")


def main():
    repo = Path(__file__).resolve().parents[1]
    extract(
        "/home/sg/Downloads/nature_pack.obj",
        repo / "models/landscape_tree/tree_naturepack.obj",
        {"Oak_tree_Cube.001"},
        "tree_naturepack.mtl",
        4.0,
        {"Trunk": "tree_trunk", "Leaves": "tree_leaves"},
    )
    extract(
        "/home/sg/Downloads/Bushes.obj",
        repo / "models/landscape_bush/bush_download.obj",
        {"Icosphere", "Icosphere.001", "Icosphere.002", "Icosphere.003", "Icosphere.004", "Icosphere.005"},
        "bush_download.mtl",
        1.0,
        {"Material.001": "bush_leaves"},
    )


if __name__ == "__main__":
    main()
