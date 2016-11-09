#Homework 1
* modify device driver program
* show video on youtube
* upload source code on github

##Installation
```
make (compile)
sudo insmod hw.ko
sudo mknod /dev/hw0 c 243 0
sudo chmod 666 /dev/hw0 (change file type)

```

##Check
```
tail /var/log/messages (display last few lines)
cat /dev/hw0 (read data from device)
echo 1 > /dev/hw0
echo 0 > /dev/hw0
```

##Remove
```
sudo rm /dev/hw0 (remove files)
sudo rmmod hw0 (remove module)
make clean (reset to before compile)
```
