
SHELL=/bin/bash
SERIAL_PORT ?= /dev/ttyACM1



flash:
	cargo run-arm --release

deploy:
	make flash
	make print

print:
	$(info blocking on ${SERIAL_PORT} )
	$(shell until [ -e "${SERIAL_PORT}" ]; do  	  sleep 0.01;	done)
	defmt-print  -e ./target/thumbv8m.main-none-eabihf/release/firmware serial --path "${SERIAL_PORT}"
