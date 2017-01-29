
arduino-builder -compile -logger=machine -hardware /usr/share/arduino/hardware \
-hardware /home/berocs/.arduino15/packages -hardware /home/berocs/sketchbook/hardware \
-tools /usr/share/arduino/tools-builder -tools /home/berocs/.arduino15/packages \
-built-in-libraries /usr/share/arduino/libraries -libraries /home/berocs/git/megadoor/lib \
-fqbn=arduino:avr:diecimila:cpu=atmega328 -ide-version=10612 -build-path /tmp/arduino_build_937528 \
-warnings=all -prefs=build.warn_data_percentage=75 \
-prefs=runtime.tools.avr-gcc.path=/home/berocs/.arduino15/packages/arduino/tools/avr-gcc/4.9.2-atmel3.5.3-arduino2 \
-prefs=runtime.tools.avrdude.path=/home/berocs/.arduino15/packages/arduino/tools/avrdude/6.3.0-arduino6 \
-verbose /home/berocs/git/megadoor/megadoor/megadoor.ino
