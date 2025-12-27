# Tạo module (mapf_solver)
pybind11_add_module(mapf_solver binding.cpp)

# Link với thư viện C++ tĩnh
target_link_libraries(mapf_solver PRIVATE mapfsolver_lib)

# # --- PHẦN TỰ ĐỘNG XỬ LÝ DLL CHO WINDOWS/MINGW ---
# if(WIN32 AND MINGW)
#     # 1. Tìm ra thư mục chứa trình biên dịch (VD: C:/msys64/mingw64/bin)
#     get_filename_component(COMPILER_BIN_PATH ${CMAKE_CXX_COMPILER} DIRECTORY)
    
#     message(STATUS "MinGW Bin Path detected: ${COMPILER_BIN_PATH}")

#     # 2. Danh sách các file DLL "sống còn" cần thiết
#     # Lưu ý: libgcc có thể là seh (64bit) hoặc dw2 (32bit), ta cứ liệt kê cả 2, có cái nào copy cái đó
#     set(MINGW_DLLS 
#         "libstdc++-6.dll" 
#         "libwinpthread-1.dll"
#         "libgcc_s_seh-1.dll" 
#         "libgcc_s_dw2-1.dll"
#     )

#     # 3. Tạo lệnh Copy tự động sau khi Build (POST_BUILD)
#     foreach(DLL_NAME ${MINGW_DLLS})
#         # Chỉ copy nếu file tồn tại trong thư mục bin của MinGW
#         if(EXISTS "${COMPILER_BIN_PATH}/${DLL_NAME}")
#             add_custom_command(TARGET mapf_solver POST_BUILD
#                 COMMAND ${CMAKE_COMMAND} -E copy_if_different
#                 "${COMPILER_BIN_PATH}/${DLL_NAME}"
#                 $<TARGET_FILE_DIR:mapf_solver>
#                 COMMENT "Auto-copying ${DLL_NAME} to output directory..."
#             )
#         endif()
#     endforeach()
# endif()