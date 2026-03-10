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

## Note:
**robot_capsule_generator:** an executable which generates a capsule given a mesh
