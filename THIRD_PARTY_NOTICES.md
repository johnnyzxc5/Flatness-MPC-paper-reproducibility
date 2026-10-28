# Third-party materials

The included six-joint Gen3 URDF is from Kinova Robotics, ros_kortex commit `c3280d8a6a6b5d96590d51538e3f265710a6c77d`, SHA-256 `eb3562ad3871cc9d0923e6f778592653bb766391ee72e536a0297a7863d00b13`.
Its BSD 3-Clause notice is retained alongside the file. The renderer does not require the visual meshes.

The numerical kernel and prediction assembly were implemented in this research project. Ideas of shared kinematics and precomputation were informed by the authors' SHAPE work; the principal simulator does not require that repository or the Modern Robotics library. Frozen Windows MEX kernels were generated from the included MATLAB sources. Rebuilding them requires MATLAB Coder and a configured compiler. Optimization Toolbox supplies quadprog; Robotics System Toolbox is used for model creation and independent validation.
