option( PLUGIN_STANDARD_QM3C2 "Install qM3C2 plugin" OFF )

if( PLUGIN_STANDARD_QM3C2 )

	project( QM3C2_PLUGIN )

        set(M3C2_PLUGIN_VERSION "0.1+")
        AddPlugin( NAME ${PROJECT_NAME} )

        target_include_directories( ${PROJECT_NAME} PRIVATE
            C:/Users/PaulLeroy/Documents/DEV/boost_1_77_0
            )

	add_subdirectory( include )
	add_subdirectory( src )
	add_subdirectory( ui )

        #================
        # git commit hash
        # Get the current working branch
        execute_process(
            COMMAND git rev-parse --abbrev-ref HEAD
            WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
            OUTPUT_VARIABLE GIT_BRANCH_CC
            OUTPUT_STRIP_TRAILING_WHITESPACE)

        # Get the latest commit hash
        execute_process(
            COMMAND git rev-parse --short HEAD
            WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
            OUTPUT_VARIABLE GIT_COMMIT_HASH_CC
            OUTPUT_STRIP_TRAILING_WHITESPACE)

        message(${PROJECT_NAME} " " GIT_BRANCH_CC " " ${GIT_BRANCH_CC})
        message(${PROJECT_NAME} " " GIT_COMMIT_HASH_CC " " ${GIT_COMMIT_HASH_CC})
        message(${PROJECT_NAME} " " QM3C2_VERSION " " ${M3C2_PLUGIN_VERSION})

        target_compile_definitions(${PROJECT_NAME} PRIVATE
            GIT_BRANCH_CC="${GIT_BRANCH_CC}"
            GIT_COMMMIT_HASH_CC="${GIT_COMMIT_HASH_CC}"
            QM3C2_VERSION="${M3C2_PLUGIN_VERSION}"
            )

endif()
