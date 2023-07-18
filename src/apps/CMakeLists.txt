add_executable(${PROJECT_NAME}_eskf main_eskf.cpp  
                               common_utility.cpp)
target_link_libraries(${PROJECT_NAME}_eskf 
                         ${PROJECT_NAME}.common
                         ${PROJECT_NAME}.pre
                         ${PROJECT_NAME}.liw
                         ${PROJECT_NAME}.algo
                         ${third_party_libs}
                        )

add_executable(${PROJECT_NAME}_eskf_rosbag main_eskf_rosbag.cpp  
                        common_utility.cpp)
target_link_libraries(${PROJECT_NAME}_eskf_rosbag 
                  ${PROJECT_NAME}.common
                  ${PROJECT_NAME}.pre
                  ${PROJECT_NAME}.liw
                  ${PROJECT_NAME}.algo
                  ${PROJECT_NAME}.tools
                  ${third_party_libs}
                 )



add_executable(test_analytic_factor test_analytic_factor.cpp )
target_link_libraries(test_analytic_factor
            ${PROJECT_NAME}.common
            ${PROJECT_NAME}.pre
            ${PROJECT_NAME}.liw
            ${PROJECT_NAME}.algo
            ${third_party_libs}
        )