# Invoked at build time (cmake -P, with -DLINK_PATH=... -DTARGET_PATH=...)
# from CMakeLists.txt's bvr_sim POST_BUILD step. Exists only because
# Windows mklink treats a bare "/" as a switch prefix -- an unconverted
# forward-slash path like "C:/Users/..." (which is what CMake generator
# expressions like $<TARGET_FILE_DIR:...> always resolve to) makes mklink
# fail with "Invalid switch - "Users"." file(TO_NATIVE_PATH) has to run
# here, at script-execution time, since by this point LINK_PATH/TARGET_PATH
# are already-resolved plain strings (the generator expression substitution
# happened before this script was invoked) -- CMakeLists.txt itself can't
# TO_NATIVE_PATH a generator expression at configure time, it isn't a real
# path yet then.
if(CMAKE_HOST_WIN32)
    file(TO_NATIVE_PATH "${LINK_PATH}" link_native)
    file(TO_NATIVE_PATH "${TARGET_PATH}" target_native)
    execute_process(COMMAND cmd /c mklink /J "${link_native}" "${target_native}" RESULT_VARIABLE result)
    if(NOT result EQUAL 0)
        message(FATAL_ERROR "mklink /J \"${link_native}\" \"${target_native}\" failed (exit ${result})")
    endif()
else()
    file(CREATE_LINK "${TARGET_PATH}" "${LINK_PATH}" SYMBOLIC)
endif()
