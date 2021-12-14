option( PLUGIN_STANDARD_QM3C2 "Install qM3C2 plugin" OFF )

if( PLUGIN_STANDARD_QM3C2 )

	project( QM3C2_PLUGIN )

        set(CMAKE_VERBOSE_MAKEFILE ON)

        AddPlugin( NAME ${PROJECT_NAME} )

        target_include_directories( ${PROJECT_NAME} PRIVATE
            C:/Users/PaulLeroy/Documents/DEV/boost_1_77_0
            )

	add_subdirectory( include )
	add_subdirectory( src )
	add_subdirectory( ui )

endif()
