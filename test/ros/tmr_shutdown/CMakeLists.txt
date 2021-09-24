# Shutdown test is skipped when the OS is Windows, for it doesn't support unix signal handling. CTRL_BREAK_EVENT will striaght up
# kill the whole test as if we press CTRL + C in the command prompt that runs unit test.
#
# The way Microsoft deal with it is via taskkill, see https://github.com/ms-iot/ros_comm/blob/init_windows/tools/roslaunch/src/roslaunch/nodeprocess.py#L449.
# However, according to the comment in https://stackoverflow.com/questions/55145395/my-application-task-cant-be-killed-by-cmd, it seems that taskkill will
# always terminate process, which is different from linux, where the process can be interrupted, terminated, and killed separately.
#
# TODO: check the returncode of subprocess if the process hanged and ultimately killed by taskkill
if (NOT WIN32)
  add_rostest(tmr_shutdown_rostest.test)
endif ()
