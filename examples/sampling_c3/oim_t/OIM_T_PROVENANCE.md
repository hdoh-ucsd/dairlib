# OIM T object identity

The execution model is
`examples/sampling_c3/urdf/oim_xarm6_tabletop/t_block.sdf`. Its two solids are
a direct dimensional translation of the OIM `tee_sampling_c3plus.xml`; they
are not the larger historical DAIRLab Push-T model.

| Property | OIM source | Drake SDF |
|---|---:|---:|
| Crossbar size | 89.0 x 19.8 x 59.6 mm | 89.0 x 19.8 x 59.6 mm |
| Crossbar center | (0, +9.9, 0) mm | (0, +9.9, 0) mm |
| Stem size | 19.8 x 79.4 x 59.6 mm | 19.8 x 79.4 x 59.6 mm |
| Stem center | (0, -39.7, 0) mm | (0, -39.7, 0) mm |
| Component masses | 0.05, 0.05 kg | 0.05, 0.05 kg equivalent |
| Total mass | 0.100 kg | 0.100 kg |
| Composite center of mass | (0, -14.9, 0) mm | (0, -14.9, 0) mm |
| Composite inertia diagonal | inferred from the two boxes | [0.000119007, 0.000064239, 0.000124043] kg m2 |
| Sliding friction | 0.30 | 0.30 |
| Start SE(2) | (0.381, +0.400, 0) | (0.381, +0.400, 0) |
| Goal SE(2) | (0.381, -0.400, pi) | (0.381, -0.400, 3.1416) |

The composite inertia is the exact parallel-axis sum of the two equal-mass
boxes. The SVG top view is generated from these same dimensions. Drake does
not implement MuJoCo rolling/torsional friction, so one-to-one here means body
geometry, mass properties, pose, and supported Coulomb sliding friction—not
identity between the two physics engines.
