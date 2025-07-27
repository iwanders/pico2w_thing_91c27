
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

partition-static-files-upload:
	# Numbers must match the values in the partition table.
	cargo pack_assets
	picotool load -p 1 -u -v -t bin /tmp/static_files.bin

# Sorcery to allow `make test -- --nocapture` to avoid typing this out of reflex and it not working.
# https://stackoverflow.com/a/14061796
ifeq (test,$(firstword $(MAKECMDGOALS)))
  # use the rest as arguments for "run"
  RUN_ARGS := $(wordlist 2,$(words $(MAKECMDGOALS)),$(MAKECMDGOALS))
  # ...and turn them into do-nothing targets
  $(eval $(RUN_ARGS):;@:)
endif

test:
	cargo unittest -- $(RUN_ARGS)
