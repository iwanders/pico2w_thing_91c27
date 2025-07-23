
SHELL=/bin/bash
SERIAL_PORT ?= /dev/ttyACM1

reboot:
	picotool reboot

flash:
	cargo run-arm --release

deploy:
	make flash
	make print

print:
	$(info blocking on ${SERIAL_PORT} )
	$(shell until [ -e "${SERIAL_PORT}" ]; do  	  sleep 0.01;	done)
	# We may get a permission denied if we instantly connect after the port is created.
	sleep 0.1
	defmt-print  -e ./target/thumbv8m.main-none-eabihf/release/firmware serial --path "${SERIAL_PORT}"

partition:
	picotool partition create ./partitions.json /tmp/partitions.uf2
	picotool load -F /tmp/partitions.uf2
	picotool partition info -F
