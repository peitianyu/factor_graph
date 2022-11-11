# Factor Graph Optimizer

Factor graph optimizer is a minimal least squares solver for problems represented as factor graphs.

The project is very flexible and easy to use.

# Some testing can see in slam

```
test_point2point
test_point2pose
test_slam_g2o
```

# build

```
cd factor_graph && mkdir build
cd  build && cmake ..
make -j
./g2o_slam2d
```
