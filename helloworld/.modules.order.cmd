cmd_/home/phuongdnguyen/data/linux_driver_practical_labs/labs/helloworld/modules.order := {   echo /home/phuongdnguyen/data/linux_driver_practical_labs/labs/helloworld/helloworld.ko; :; } | awk '!x[$$0]++' - > /home/phuongdnguyen/data/linux_driver_practical_labs/labs/helloworld/modules.order
