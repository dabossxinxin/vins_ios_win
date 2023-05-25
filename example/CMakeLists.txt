
add_executable(example ./estimator_node.cpp)
target_link_libraries(example vins_estimator win_player)

add_executable(ios_example ./estimator_node_2.cpp)
target_link_libraries(ios_example vins_estimator win_player)