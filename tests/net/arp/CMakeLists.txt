include($ENV{ZEPHYR_BASE}/cmake/app/boilerplate.cmake NO_POLICY_SCOPE)
project(NONE)

target_include_directories(
  app
  PRIVATE
  $ENV{ZEPHYR_BASE}/subsys/net/ip
  $ENV{ZEPHYR_BASE}/subsys/net/l2/ethernet
  )
FILE(GLOB app_sources src/*.c)
target_sources(app PRIVATE ${app_sources})
