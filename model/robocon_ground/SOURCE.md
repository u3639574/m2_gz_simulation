Source: https://github.com/jimi4-lei/gazebo_models
Pinned commit: 1391b36e4707cdfc7071d6ad5509bc9a8b828603
Original: robocon_ground/meshes/Untitled.dae, copied byte for byte to venue.dae.
All visual nodes are retained, including fixed Sky Block halves Cube.020–Cube.043 and Plane.015. There are no separate dynamic Earth Block models in the upstream Gazebo world; our world supplies three. Sky Blocks remain static mesh geometry, not graspable bodies.
Collision adaptation: collision mesh retains all solid venue geometry (including Sky Blocks), omits markings; added floor box. This fixes upstream primitive collisions that do not follow the visible ramps/stairs. The original Gazebo Classic world is therefore not used verbatim.
This is the supplied community field model, not a claim of a certified competition digital twin. Upstream provides no explicit license file; attribution is retained for this requested local integration.
