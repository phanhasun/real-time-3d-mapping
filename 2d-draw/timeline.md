# Timeline

- Cài ros
- Build SDK
- Vẽ 2d không gian dựa trên dữ liệu của lidar thời gian thực
  - Cài SFML (vẽ 2d)
  - Giao tiếp với lidar bằng sdk để lấy dữ liệu

## Tìm hiểu Lidar RPLIDAR A1

- Tìm hiểu cách hoạt động của Lidar (lí thuyết).
- Tìm hiểu cách lấy dữ liệu quét từ RPLIDAR A1
  - Tìm hiểu Cách cài và sử dụng `Slamtec/rplidar_sdk`.
  - Lấy dữ liệu quét từ RPLIDAR A1 lên máy tính
- Tìm kiếm và chọn thư viện vẽ 2d trên ngôn ngữ C/C++
  - Tìm hiểu Cách cài và Sử dụng thư viện SFML
  - Vẽ hình 2d cơ bản bằng SFML
- Vẽ dữ liệu quét từ RPLIDAR A1 lên 2D SFML
  - Lọc dữ liệu thô từ RPLIDAR A1
  - Chuyển thành các dữ liệu đọc được
  - Chuyển dữ liệu thô thành tập hợp các điểm trong hệ tọa độ Cartesian
  - Vẽ các điểm lên 2D
  - Nối các điểm 2D với nhau
- Tìm hiểu module Network của thư viện SFML
  - Tìm hiểu cách sử dụng Network của thư viện SFML
  - Tìm hiểu cách sự dụng của một số công cụ Network như `ssh`, `netcat`,...
  - Viết phần mềm giao tiếp cơ bản giữa RPI3 với Máy tính.
- Tách phần mềm Render dữ liệu thành 2 phần mềm: 
  - Grabdata-server
    - Chạy trên RPI3
    - Kết nối với RPLIDAR A1
    - Kết nối wifi
    - Chạy server chờ Render-client kết nối vào
  - Render-client
    - Chạy trên máy tính
    - Kết nối tới Grabdata-server
    - Render 3d trên dữ liệu lấy từ server
- Xây dựng và lắp ráp mô hình xe
- Thêm tính năng điều khiển cho client. Set vị trí trên bản đồ 3d để xe tự động chạy tới.
