#
# Compiler flags come from parent
#
option(STA_TEST_ENABLE_VTUNE_PROFILE "Enable selective sampling in VTUNE" OFF)
option(STA_TEST_ENABLE_CALLGRIND_PROFILE "Enable selective sampling with callgrind" OFF)

#
#
# Build files configuration
#
#

#Source files for the demo executable (not generated)
file(GLOB_RECURSE STA_TEST_SOURCES *.cpp)
file(GLOB_RECURSE STA_TEST_HEADERS *.hpp)

set(LEXER_SRC ${CMAKE_CURRENT_SOURCE_DIR}/parsers/vpr_timing_graph.l)
set(PARSER_SRC ${CMAKE_CURRENT_SOURCE_DIR}/parsers/vpr_timing_graph.y)

#Include directories
foreach(header ${STA_TEST_HEADERS})
    get_filename_component(incl_dir ${header} DIRECTORY)
    list(APPEND STA_TEST_INCLUDE_DIRS ${incl_dir})
endforeach()
#Remove duplicate include directories
list(REMOVE_DUPLICATES STA_TEST_INCLUDE_DIRS)


#
# Configure intermediate files
#

#
#
# Define the actual build targets
#
#

#Define Executable
add_executable(sta_test
               ${STA_TEST_SOURCES}
               ${STA_TEST_HEADERS})

#Exectuable Includes
target_include_directories(sta_test PRIVATE
                           ${STA_TEST_INCLUDE_DIRS})

#Executable links to the library
target_link_libraries(sta_test libtatum libtatumparse)

if(STA_TEST_ENABLE_VTUNE_PROFILE)
    target_include_directories(sta_test PRIVATE /opt/intel/vtune_amplifier_xe/include)
    target_link_libraries(sta_test /opt/intel/vtune_amplifier_xe/lib64/libittnotify.a ${CMAKE_DL_LIBS})
    target_compile_definitions(sta_test PRIVATE STA_TEST_PROFILE_VTUNE=1)
    target_compile_options(sta_test PUBLIC -g)
endif()

if(STA_TEST_ENABLE_CALLGRIND_PROFILE)
    
    #To selectively profile using callgrind:
    #  valgrind --tool=callgrind --collect-atstart=no --instr-atstart=no --cache-sim=yes --cacheuse=yes ./command

    target_compile_definitions(sta_test PRIVATE STA_TEST_PROFILE_CALLGRIND=1)
    target_compile_options(sta_test PUBLIC -g)
endif()
