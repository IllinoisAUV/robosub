library(whitebalance whitebalance.cpp)
library(median_subtract median_subtract.cpp)

node(preprocess_node preprocess_node.cpp whitebalance median_subtract ${OpenCV_LIBRARIES})
