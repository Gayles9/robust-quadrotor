# Development Environment

Inventory recorded on 2026-08-12.

## Host Hardware

- Lenovo Yoga 9, model 82BG
- Intel Core i7-1195G7
- 8 GB physical RAM
- Intel Iris Xe integrated graphics
- Approximately 231 GB free on the Windows drive at inventory time

## Host and WSL

- Windows host reported Windows 10 Home with build 26200. This product-name/build pairing is inconsistent and requires later confirmation.
- WSL2 with Ubuntu 24.04.4 LTS
- WSL kernel 6.6.114.1-microsoft-standard-WSL2
- WSL reported 3.6 GiB memory and 1.0 GiB swap

## Development Tools

- Python 3.12.3
- Git 2.43.0
- GCC 13.3.0
- CMake 3.28.3
- uv 0.12.3

WSL memory may constrain simultaneous PX4/Gazebo use. This will be tested during the compatibility spike.
