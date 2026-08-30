# OIM xArm6 tabletop assets

Imported through the audited Python-port asset set at
`sim/models/oim_xarm6_tabletop`, whose upstream is
NikolaRaicevic2001/Object-Informed-Manipulation-MJX.

This C++ branch intentionally vendors only the open-table Sampling-C3+ scene,
its shared scene and T definitions, the xArm model, and the seven referenced
OBJ meshes. It does not copy the other OIM scenarios or controller YAML. The
files are unchanged from that local audited import; the `assets` symlink beside
the scene compensates for the pinned Drake MJCF parser resolving an included
model's mesh paths relative to the root scene file.
