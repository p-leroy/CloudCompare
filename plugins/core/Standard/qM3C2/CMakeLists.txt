option( PLUGIN_STANDARD_QM3C2 "Install qM3C2 plugin" OFF )

if( PLUGIN_STANDARD_QM3C2 )

	project( QM3C2_PLUGIN )

        set(CMAKE_VERBOSE_MAKEFILE ON)

        #set( Boost_NO_BOOST_CMAKE ON )
        #set( Boost_DIR C:/Users/PaulLeroy/Documents/DEV/boost_1_77_0 )
        #set( Boost_DIR C:/Users/PaulLeroy/miniconda3/Library/lib/cmake/Boost-1.73.0)

        #find_package(Boost REQUIRED)

        message(${PROJECT_NAME} " Boost_INCLUDE_DIR " ${Boost_INCLUDE_DIR} )
        message(${PROJECT_NAME} " boost_headers_DIR " ${boost_headers_DIR} )

        #include_directories(
        #    ${Boost_DIR}
        #    )

        AddPlugin( NAME ${PROJECT_NAME} )

        target_include_directories( ${PROJECT_NAME} PRIVATE
            #${CMAKE_CURRENT_LIST_DIR}/../../../../boost_1_77_0
            #${CMAKE_CURRENT_LIST_DIR}/../../../../eigen-3.4.0
            C:/Users/PaulLeroy/Documents/DEV/boost_1_77_0
            )

        #target_sources( ${PROJECT_NAME}
        #    PRIVATE
        #    ${Boost_DIR}/boost/math/distributions/students_t.hpp
        #    )

	add_subdirectory( include )
	add_subdirectory( src )
	add_subdirectory( ui )

endif()
