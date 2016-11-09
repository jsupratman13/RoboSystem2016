import os
import sys
import time

try:
	device = os.open('/dev/hw0', os.O_RDWR)
	while True:
		time.sleep(0.5)
		value = os.read(device,2)
		print value
		if int(value):
			os.write(device, '0')
		else:
			os.write(device, '1')

except (KeyboardInterrupt, SystemExit):
	os.write(device, '0')
	os.close(device)
	sys.exit()

