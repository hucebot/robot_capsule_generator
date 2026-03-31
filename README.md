# robot_capsule_generator
Tools to generate capsules from mesh files (using roboptim-capsule and trimesh2), and apply the capsule parameters to a urdf model.

## USAGE:

- ```robot_capsule_urdf input.urdf -o output.urdf```

e.g.
```robot_capsule_urdf /home/ros2_ws/src/huro/resources/description_files/urdf/g1/g1_23dof.urdf -o g1_23dof_capsule.urdf```

Creates a urdf with cylinders for collisions.


- ```robot_capsule_urdf_to_rviz output.urdf -o output2.urdf```

e.g.
```robot_capsule_urdf g1_23dof_capsule.urdf -o g1_23dof_capsule.rviz```

Creates a urdf with cylinders and spheres for collisions, using output from the previous script.

- ```./compute_default_collisions --urdf_path robot.urdf --num_trials 1000 --min_collision_fraction 0.001 --max_collision_fraction 0.95```

e.g.
```./compute_default_collisions --urdf_path /home/ros2_ws/src/tiago_dual_cartesio_config/capsules/urdf/tiago_pro_capsules.rviz --num_trials 10000```

`./compute_default_collisions` generates a file `collision_pairs.json` containing a list of link pairs that can be used in a collision avoidance constraint.
The list is obtained by considering all possible collision pairs in the robot model and removing pairs that are *always* or *never* in collision.
To determine these cases, the algorithm samples `num_trials` robot configurations and computes the distance for each collision pair. A pair is considered in collision when the distance falls below a threshold (`0.001`).
If the fraction of detected collisions is greater than or equal to `max_collision_fraction`, the link pair is classified as *always* in collision. Conversely, if the fraction of detected collisions is less than or equal to `min_collision_fraction`, the link pair is classified as *never* in collision.

## Note:

**robot_capsule_generator:** an executable which generates a capsule given a mesh


