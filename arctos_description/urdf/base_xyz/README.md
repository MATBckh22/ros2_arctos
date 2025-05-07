# arctos_description/urdf/base_xyz

This folder contains the URDF files for the base of the robot. The base is composed of the following parts:

**Joints**:
- `world_joint`: The joint that connects the world frame to the base link. (`world` to `base_link`) (fixed joint)
- `X_joint`: The joint that connects the base link to the first link of the robot. (`base_link` to `Link_1_1`)
- `Y_joint`: The joint that connects the first link of the robot to the second link. (`Link_1_1` to `Link_2_1`)
- `Z_joint`: The joint that connects the second link of the robot to the third link. (`Link_2_1` to `Link_3_1`)

## Joints and Links

### Links

| Link        | Description                     | Related Mesh    |
| ----------- | ------------------------------- | --------------- |
| `base_link` | fixed base of the robot         | `base_link.stl` |
| `link_1_1`  | Y core                          | `link_1_1.stl`  |
| `link_2_1`  | Y gearbox to Z core             | `link_2_1.stl`  |
| `link_3_1`  | Z gearbox to A core             | `link_3_1.stl`  |
| `link_4_1`  | A gearbox to B core             | `link_4_1.stl`  |
| `link_5_1`  | B gearbox to C core             | `link_5_1.stl`  |
| `link_6_1`  | Ring between C core and gripper | `link_6_1.stl`  |

### Joints

| Joint         | Parent Link | Child Link  | type       | Joint |
| ------------- | ----------- | ----------- | ---------- | ----- |
| `world_joint` | `world`     | `base_link` | `fixed`    | -     |
| `X_joint`     | `base_link` | `link_1_1`  | `revolute` | X     |
| `Y_joint`     | `link_1_1`  | `link_2_1`  | `revolute` | Y     |
| `Z_joint`     | `link_2_1`  | `link_3_1`  | `revolute` | Z     |
| `A_joint`     | `link_3_1`  | `link_4_1`  | `revolute` | A     |
| `B_joint`     | `link_4_1`  | `link_5_1`  | `revolute` | B     |
| `C_joint`     | `link_5_1`  | `link_6_1`  | `revolute` | C     |