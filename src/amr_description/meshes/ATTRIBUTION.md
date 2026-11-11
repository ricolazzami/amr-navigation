# Third-party assets

## `meshes/create_body.dae` + `materials/textures/create_body.png`

iRobot Create body mesh, from the [Gazebo model
database](https://github.com/osrf/gazebo_models) (`create` model).

- Source: <https://github.com/osrf/gazebo_models/blob/master/create/meshes/create_body.dae>
- Texture: <https://github.com/osrf/gazebo_models/blob/master/create/materials/textures/create_body.png>
- Author: Open Source Robotics Foundation (OSRF), as part of the Gazebo model database
- Licence: Creative Commons Attribution 3.0 Unported (CC-BY 3.0),
  <https://creativecommons.org/licenses/by/3.0/>
- Changes: none to the asset. Referenced from `urdf/amr.urdf.xacro` with a
  `0.0254 0.0254 0.0254` scale because the mesh is authored in inches
  (12.99 in = 0.330 m). 11,280 triangles.

The mesh is a **visual shell only**: the link's `<collision>`, `<inertial>` and
the `gz-sim-diff-drive-system` parameters are the project's own numbers and are
not derived from this asset.
