#Device driver test

##Method
```
make (compile)
sudo insmod myled.ko (insert to modules)
sudo mknod /dev/myled0 c 243 0 (manually create device file)
sudo chmod 666 /dev/myled0 (change file type)

```

##Check
```
lsmod (list modules)
modinfo (check modules info)
tail /var/log/messages (display last few lines)
cat /proc/devices | grep myled (view file)
fdls -l /dev/myled0 (list files, check type)
ls -l /sys/class/myled/ (check to see if myled directory is created)
ls -l /sys/class/myled/myled0/
cat /sys/class/myled/myled0/dev (check major and minor number)
ls -l /dev/myled0 (check if device file is made)
cat /dev/myled0 (read data from device)
echo abc > /dev/myled0 (send data to device check in tail. 0 off, 1 on)
```

##remove
```
sudo rm /dev/myled0 (remove files)
sudo rmmod myled (remove module)
make clean (reset to before compile)
```
