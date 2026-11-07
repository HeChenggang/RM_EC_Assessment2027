# 固件工程

`chassis/` 为底盘 C 板工程，`gimbal/` 为独立云台 C 板工程，均面向 STM32F407IGH6。
云台构建步骤、引脚与当前运行限制见 [gimbal/README.md](gimbal/README.md)。
保留 CubeMX `.ioc`、启动文件、链接脚本和必要的工程配置；构建产物由根目录 `.gitignore` 排除。

导入后记录：MCU、IDE / 编译器版本、HAL / RTOS 版本、构建命令、下载方式及引脚分配。
