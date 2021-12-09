
target_sources( ${PROJECT_NAME}
        PRIVATE
		${CMAKE_CURRENT_LIST_DIR}/qM3C2.h
		${CMAKE_CURRENT_LIST_DIR}/qM3C2Commands.h
		${CMAKE_CURRENT_LIST_DIR}/qM3C2Dialog.h
		${CMAKE_CURRENT_LIST_DIR}/qM3C2DisclaimerDialog.h
		${CMAKE_CURRENT_LIST_DIR}/qM3C2Process.h
		${CMAKE_CURRENT_LIST_DIR}/qM3C2Tools.h
                ${CMAKE_CURRENT_LIST_DIR}/students_t_distribution_critical_values.h
)

target_include_directories( ${PROJECT_NAME}
	PRIVATE
		${CMAKE_CURRENT_SOURCE_DIR}
)
