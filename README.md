# 写一个简单的因子图库

[peitianyu/factor_graph (github.com)](https://github.com/peitianyu/factor_graph)

# 简介

```
此程序主要学习了mini_sam思路,讲真mini_sam库真心清晰,明了,干净.
设计目的:
	实现一种开放的,便于拓展的因子图库,core函数用模板实现,为实现不同的因子图,可自行设计variable与factor,本案例提供了4种方案
程序包含
Core:	主函数在graph_optimize进行优化,然后通过sparsity_pattern保存稀疏矩阵缓存,variable与factor构造虚函数
types: 	构造二维空间下的variable与factor
slam:	简单的写了几个在二维下做的因子图
test: 	测试主函数
```

# 架构

```
variable: 			相当于图优化里边的顶点
factor: 			通过variable构造成一元边或者二元边, 并且将求解Jacobian也放在此类中(用的是数值求解雅各比)
factor_graph: 		将构造好的variable与factor添加入因子图,由于之后的优化
graph_optimize: 	优化构造好的因子图,这里由于稀疏化技巧所以分出一个sparsity_pattern稀疏缓存器模块
sparsity_pattern: 	包含两个部分一个是variable搜索引导一个是H与b稀疏矩阵缓存
```

# 测试

```
主要测试了:
1. 纯2d_point的优化
2. 2d环境下point与pose的优化(比如, 反光板与里程计)
3. 纯2d_pose的优化,(主要展示了读取g2o文件,并优化的效果)
4. 纯3d_pose的优化,(主要展示了读取g2o文件,并优化的效果)
```

# BUG

```
1. 加入sqr_info后发现优化速度明显变慢,没找的什么原因
```

# TODO

```
1. 边缘化操作
2. 如果H并不是稀疏矩阵,通过近似最小度排序变为稀疏矩阵
3. 加入imu预积分因子
```

# Build

```
cd factor_graph_optimize && mkdir build
cd  build && cmake ..
make -j
./g2o_slam2d
```
