# Fetch the Pico SDK if not present
include(FetchContent)
set(FETCHCONTENT_QUIET FALSE)
FetchContent_Declare(
  pico_sdk
  GIT_REPOSITORY https://github.com/raspberrypi/pico-sdk.git
  GIT_TAG master
)
FetchContent_MakeAvailable(pico_sdk)
