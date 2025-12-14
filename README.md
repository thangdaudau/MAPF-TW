# Multi-Agent Path Finding (MAPF) with Time Windows

Dự án triển khai một số thuật toán giải quyết bài toán Tìm đường đa tác nhân (MAPF) được sửa đổi để phù hợp với hàm mục tiêu của bài toán MAPF-TW sử dụng Core C++ hiệu năng cao kết hợp với giao diện Python.

> **⚠️ LƯU Ý QUAN TRỌNG TRƯỚC KHI BẮT ĐẦU:**
> 
> - Dự án này **BẮT BUỘC** chạy trên môi trường **MSYS2 / MinGW64**.
>     
> - Tuyệt đối **KHÔNG dùng `pip`** để cài các thư viện nặng `(numpy, pygame...)` nhằm tránh lỗi xung đột DLL. Chúng ta sẽ dùng `pacman`.
>     
> - Hãy đảm bảo bạn đã cài đặt **MSYS2** và đang mở **MSYS MinGW64 Shell**.
>     

## 1. Cài đặt môi trường (Installation)

Thực hiện lần lượt các bước sau trong **Terminal MinGW64**:

### Bước 1: Cài đặt thư viện hệ thống (Pacman)

Copy và chạy lệnh sau để cài đặt Python, trình biên dịch C++ và các thư viện phụ thuộc:

Bash

```
pacman -S --needed --noconfirm \
    mingw-w64-x86_64-toolchain \
    mingw-w64-x86_64-cmake \
    mingw-w64-x86_64-python \
    mingw-w64-x86_64-python-numpy \
    mingw-w64-x86_64-python-matplotlib \
    mingw-w64-x86_64-python-pygame \
    mingw-w64-x86_64-python-yaml \
    mingw-w64-x86_64-python-pillow \
    mingw-w64-x86_64-python-opencv \
    mingw-w64-x86_64-python-tkinter
```

### Bước 2: Tạo môi trường ảo (Virtual Environment)

Chúng ta tạo `.venv` để quản lý dự án, nhưng dùng cờ `--system-site-packages` để nó "nhìn thấy" các thư viện `(numpy, pygame...)` đã cài bằng `pacman` ở trên.

1. **Tạo `venv` (Tại thư mục gốc dự án):**
    
    Bash
    
    ```
    python -m venv .venv --system-site-packages
    ```
    
1. **Kích hoạt `venv`:**
    
    Bash
    
    ```
    source .venv/bin/activate
    ```
    
    _Dấu hiệu thành công: Bạn sẽ thấy chữ `(.venv)` xuất hiện ở đầu dòng lệnh._
    

## 2. Cấu hình VS Code

Sau khi cài đặt xong môi trường, hãy mở VS Code tại thư mục dự án (`code .`) và thiết lập:

1. **Cài Extensions:**
    
    - `C/C++` (Microsoft)
        
    - `CMake Tools` (Microsoft)
        
    - `Python` (Microsoft)
        
2. **Chọn Python Interpreter:**
    
    - Nhấn `Ctrl` + `Shift` + `P` -> Gõ `Python: Select Interpreter`.
        
    - Chọn dòng có đường dẫn trỏ tới thư mục `.venv` vừa tạo.
        
        - _Ví dụ:_ `./.venv/bin/python` hoặc `./.venv/Scripts/python.exe`.
            

## 3. Hướng dẫn Build

Dự án dùng CMake Tools để biên dịch.

1. **Cấu hình CMake:**
    
    - Nhấn `Ctrl` + `Shift` + `P` -> Gõ `CMake: Configure`.
        
    - Chọn Kit: **GCC ... x86_64-w64-mingw32** (Chọn đúng bản GCC của MinGW64, không chọn Visual Studio).
        
2. **Biên dịch (Build):**
    
    - Nhấn phím `F7` (hoặc nút **Build** ở thanh trạng thái dưới đáy màn hình).
        

**Kết quả:** Nếu build thành công, sẽ tạo ra trong thư mục `build/` cùng các thư viện và module trong đó.

## 4. Cách sử dụng (Usage)

Đảm bảo bạn đã kích hoạt môi trường ảo (`source .venv/bin/activate`) trước khi chạy lệnh.

### A. Solver (Chạy thuật toán tìm đường)

File `main.py` nằm ở thư mục gốc.

Bash

```
python main.py <ALGO> <MAP_NAME>
```

- **ALGO:**
    
    - `ILP`: Integer Linear Programming (Tối ưu, chậm).
        
    - `CBS`: Conflict-Based Search (Tối ưu).
        
    - `ICBS`: Improved CBS (Tối ưu, hiệu quả cao. Nhưng chưa triển khai hoàn thiện, phần low level đang dùng tạm GCBS).
        
    - `GCBS`: Greedy CBS (Cực nhanh, Suboptimal, không đảm bảo chất lượng giải pháp).
        
- **MAP_NAME:** Tên file map trong thư mục `res/` (ví dụ nhập `6x6x6` sẽ chạy file `res/6x6x6.input.yaml`).
    

**Ví dụ:**

Bash

```
python main.py GCBS 6x6x6
```

_(Kết quả output sẽ được lưu vào `res/6x6x6.output.yaml`)_

### B. Visualizer (Xem mô phỏng)

Các công cụ hiển thị nằm trong thư mục `visualize/`.

- **Pygame Visualizer (Khuyên dùng - Mượt, có xuất Video):**
    
    Bash
    
    ```
    python visualize/visualizer_smooth.py 6x6x6
    ```
    
- **Matplotlib Visualizer (Cũ):**
    
    Bash
    
    ```
    python visualize/visualizer.py 6x6x6
    ```
    

### C. Map Editor (Tạo map thủ công)

Công cụ vẽ map để tạo dữ liệu test.

Bash

```
python visualize/drawer.py
```

- **Phím tắt:** `1` (Tường), `2` (Agent), `3` (Goal), `4` (Xóa).
    
- **Lưu:** Nhấn nút `Save Map` để lưu file `.yaml` vào thư mục `res/`.
    

## 5. Cấu trúc thư mục

Plaintext

```
MAPF-TW/
├── .venv/                  # Môi trường ảo (Kế thừa gói hệ thống MinGW64)
├── build/                  # Chứa file biên dịch .pyd sau khi build
├── core/                   # Chứa thuật toán (đang phát triển)
├── res/                    # Input (.input.yaml) và Output (.output.yaml)
├── visualize/              # Các công cụ giao diện
│   ├── drawer.py           # Tool vẽ map
│   ├── visualizer.py       # Visualizer cũ (Matplotlib)
│   └── visualizer_smooth.py # Visualizer mới (Pygame)
├── main.py                 # Script chính chạy thuật toán
├── utils.py                # Các hàm tiện ích Python
└── CMakeLists.txt          # Cấu hình Build cho CMake
```