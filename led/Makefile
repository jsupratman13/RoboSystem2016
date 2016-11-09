obj-m:= myled.o #define object file

myled.ko: myled.c
	make -C /usr/src/linux M=`pwd` V=1 modules
clean:
	make -C /usr/src/linux M=`pwd` V=1 clean
