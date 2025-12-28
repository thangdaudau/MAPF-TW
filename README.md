# Multi-Agent Path Finding (MAPF) with Time Windows

Clone dự án này:

```
git clone --recurse-submodules https://github.com/thangdaudau/MAPF-TW.git
```

Dự án triển khai một số thuật toán giải quyết bài toán Tìm đường đa tác nhân (MAPF) được sửa đổi để phù hợp với hàm mục tiêu của bài toán MAPF-TW sử dụng Core C++ hiệu năng cao kết hợp với giao diện Python.

> **⚠️ LƯU Ý QUAN TRỌNG TRƯỚC KHI BẮT ĐẦU:**
> 
>- Dự án này **BẮT BUỘC** chạy trên môi trường **WSL2 – Ubuntu 22.04 LTS 2** (24.04 lỗi `systemd` cay `vl`)
>   
>- Toàn bộ thư viện **C++  cài bằng `apt`**
>    
>- Visualization chạy qua **WSLg** (Windows 11)
>
>- Giao diện trực quan hóa hơi tật nhưng có là được.

## 1. Cài đặt môi trường (Installation)

Thực hiện các lệnh sau **bên trong WSL Ubuntu 22.04**.

### Bước 1: Cập nhật hệ thống

Bash

```
sudo apt update && sudo apt upgrade -y
```

### Bước 2: Cài compiler C++, CMake, Boost và WSLg

Bash

```
sudo apt install -y \
    build-essential \
    cmake \
    gdb \
    libboost-all-dev \
    x11-apps
```

### Bước 3: Cài Python cơ bản (system)

Bash

```
sudo apt install -y \
    python3 \
    python3-dev \
    python3-venv \
    python3-pip \
    python3-tk
```

### Bước 4: Tạo và kích hoạt Virtual Environment

Tại thư mục gốc dự án:

```
python3 -m venv .venv
source .venv/bin/activate
```

### Bước 5: Cài thư viện Python bằng pip

```
pip install --upgrade pip
pip install \
    numpy \
    matplotlib \
    pygame \
    pyyaml \
    opencv-python \
    keras \
    tensorflow
```

> ⚠️ **KHÔNG cài Python libs bằng `apt`**  
> → Tránh trộn ABI, tránh bug ngầm khi debug C++ ↔ Python.
    

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
        
    - Chọn Kit: **GCC ... x86_64-linux-gnu** (Chọn đúng bản GCC của WSL)
        
2. **Biên dịch (Build):**
    
    - Nhấn phím `F7` (hoặc nút **Build** ở thanh trạng thái dưới đáy màn hình).
        

**Kết quả:** Nếu build thành công, sẽ tạo ra trong thư mục `build/` cùng các thư viện và module trong đó.

## 4. Cách sử dụng (Usage)

Đảm bảo bạn đã kích hoạt môi trường ảo (`source .venv/bin/activate`) trước khi chạy lệnh.

### A. Solver (Chạy thuật toán tìm đường)

File `main.py` nằm ở thư mục gốc.

Bash

```
python3 main.py <ALGO> <MAP_NAME>
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
python3 main.py GCBS 6x6x6
```

_(Kết quả output sẽ được lưu vào `res/6x6x6.output.yaml`)_

### B. Visualizer (Xem mô phỏng)

Các công cụ hiển thị nằm trong thư mục `visualize/`.

- **Pygame Visualizer (Khuyên dùng - Mượt, có xuất Video):**
    
    Bash
    
    ```
    python3 visualize/visualizer_smooth.py 6x6x6
    ```
    
- **Matplotlib Visualizer (Cũ):**
    
    Bash
    
    ```
    python3 visualize/visualizer.py 6x6x6
    ```
    

### C. Map Editor (Tạo map thủ công)

Công cụ vẽ map để tạo dữ liệu test.

Bash

```
python3 visualize/drawer.py
```

- **Phím tắt:** `1` (Tường), `2` (Agent), `3` (Goal), `4` (Xóa).
    
- **Lưu:** Nhấn nút `Save Map` để lưu file `.yaml` vào thư mục `res/`.
    

## 5. Cấu trúc thư mục

Plaintext

```
MAPF-TW/
├── build/                  # Chứa file biên dịch .pyd/.so sau khi build
├── core/                   # Chứa thuật toán (đang phát triển)
├── res/                    # Input (.input.yaml) và Output (.output.yaml)
├── visualize/              
│   ├── drawer.py           # Tool vẽ map
│   ├── visualizer.py       # Visualizer cũ (Matplotlib)
│   └── visualizer_smooth.py # Visualizer mới (Pygame)
├── main.py                 
├── utils.py                
└── CMakeLists.txt          # Cấu hình Build cho CMake
```