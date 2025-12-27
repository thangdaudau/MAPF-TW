cmake_minimum_required(VERSION 3.20)
project(MAPFSolver)
option(MAPFSOLVER_BUILD_EXE "Build MAPFSolver executable" ON)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
add_compile_options(-O2 -Wall)

# 1. Tìm TẤT CẢ file .cpp trong dự án (bao gồm cả main và các file trong subfolder)
file(GLOB_RECURSE ALL_SOURCES 
  ${PROJECT_SOURCE_DIR}/*.cpp
)

# Loại bỏ các file rác trong thư mục build (quan trọng để không build nhầm file tạm)
list(FILTER ALL_SOURCES EXCLUDE REGEX ".*/build/.*")

# 2. Tạo danh sách source cho LIBRARY (Tất cả trừ main.cpp)
# Copy danh sách gốc sang danh sách mới để xử lý
set(LIB_SOURCES ${ALL_SOURCES})
# Lọc bỏ file main.cpp ra khỏi danh sách này
list(FILTER LIB_SOURCES EXCLUDE REGEX ".*main\\.cpp$")

# 3. Tạo Static Library từ danh sách đã lọc (KHÔNG CHỨA main.cpp)
add_library(mapfsolver_lib STATIC 
  ${LIB_SOURCES}
)

# Thiết lập đường dẫn include cho thư viện (để các file trong lib tìm thấy nhau)
target_include_directories(mapfsolver_lib
  PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
)

if (MAPFSOLVER_BUILD_EXE) 
  # 4. Tạo Executable CHỈ từ main.cpp
  add_executable(${PROJECT_NAME} 
    main.cpp
  )

  # Link thư viện vào executable
  target_link_libraries(${PROJECT_NAME}
    PRIVATE
      mapfsolver_lib
  )
endif()