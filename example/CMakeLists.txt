
add_executable(example ./estimator_node.cpp)
target_link_libraries(example vins_estimator win_player rabbitmq::rabbitmq-static)
target_link_libraries(example debug ${JSON_DEBUG_LIB} optimized ${JSON_RELEASE_LIB})

add_executable(ios_example ./estimator_node_2.cpp)
target_link_libraries(ios_example vins_estimator win_player rabbitmq::rabbitmq-static)
target_link_libraries(ios_example debug ${JSON_DEBUG_LIB} optimized ${JSON_RELEASE_LIB})