# External dependencies and attribution

The MIT license covers this project's original source and generated seed data,
not independently licensed dependencies.

- [LegoTechnicSimulation](https://github.com/yoff/LegoTechnicSimulation) reconstructs
  mechanical assemblies from LDraw. Its exact source revision is pinned in
  `pyproject.toml`; install it separately under its own license.
- [LDraw](https://www.ldraw.org/) supplies the parts library. Download it separately
  and observe the library's license and attribution requirements. No part meshes,
  LDraw library files or downloaded packages are distributed in this repository.
  The seed JSON files reference part identifiers and placements, not part meshes.
- [MuJoCo](https://github.com/google-deepmind/mujoco) supplies rigid-body dynamics.
  Python/.NET packages retain their respective licenses.
- The XL motor curve is based on
  [Philo's regulated-9 V measurements](https://www.philohome.com/motors/motorcomp.htm).
  The motor model records these sources and its limitations.

LEGO and Technic are trademarks of the LEGO Group. This independent research
project is not affiliated with or endorsed by the LEGO Group.
