# nms_rotated

The func in this repo is an optimized version of nms_rotated. And the c++ func has been bound to a python func by pybind.

Excute this to compile and install module:

```bash
python3 setup.py build_ext --inplace
```
Excute this to run the test program:

```bash
python3 test.py
```
Briefly, this optimization takes the step of calculating the relative coordinates of two points out of the nms's comparison box loop and compute all the absolute coordinates once, omitting the center shift step. From the experiment, it is known that this optimization does not have too much loss of accuracy, because the input pictures that model inferences are generally from 1080p to 4k resolution.

For details of this optimization method, [please see this (chinese version)](./docs/rotated_nms.pdf)

