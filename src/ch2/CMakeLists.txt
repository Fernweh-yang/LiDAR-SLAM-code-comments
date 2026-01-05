 add_executable(motion motion.cc)
 target_link_libraries(motion
	PRIVATE
        spdlog::spdlog
        ${PROJECT_NAME}.common
        ${PROJECT_NAME}.tools
 )