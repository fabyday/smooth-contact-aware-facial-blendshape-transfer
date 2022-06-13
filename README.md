# smooth-contact-aware-facial-blendshape-transfer
unofficial implementation. https://dl.acm.org/doi/10.1145/2491832.2491836






# deps
manually install below library
[PCL -1.12.1](https://github.com/PointCloudLibrary/pcl/releases/tag/pcl-1.12.1) kdtree


add it to your system environment path ```path/to/pcl/bin```
or ```set env in cmake```


auto installed libraries
```
libigl  
eigen  
spdlog  
yaml-cpp  
```


# TODO 
- [x]task1 : deformation transfer
- [O] find correspondence 
- [-] find 
- task2 : add triangle constraints
- task3 : add laplacian constraints


# LOGGING
for Eigen testing Use 
PRETTY_LOG_BEGIN(msg, category), PRETTY_LOG_END(msg, category), log_sparse(), log_dense()

```c++

#include <Eigen/Sparse>
#include <Eigen/Core>
#include "pretty_log.h"

// if you disable it. it doesn't write or execute anything 
// between PRETTY_LOG_BEGIN and PRETTY_LOG_END
#define category "category_name" 
PRETTY_LOG_BEGIN("test", "DENSE_WRITE")
your code....
PRETTY_LOG_END("test", "DENSE_WRITE")

// or 
Eigen::SparseMatrix<float> sp;
log_sparse(sp)

//or
Eigen::Matrix<float> mat;
log_dense(mat)

```


