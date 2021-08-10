# HP 3468a HP-IL to serial decoder

This is a simplified HP IL decoder (read only). HP3468A is put into "talk" mode
where it simply prints values constantly and doesn't require any input.

* [HP-IL introduction](http://diyhpl.us/~nmz787/hp_journal/www.hpl.hp.com/hpjournal/pdfs/IssuePDFs/1983-01.pdf)
* [a complete implementation](http://www.jeffcalc.hp41.eu/hpil/)

# Circuit

* To complete the HP IL loop device is [short-circuited back to itself](img/device-back.jpg).
* Analog [part](img/analog-schematic.png)
* MCU [connection](img/mcu-connection.png)
* E.g. [protoboard](img/protoboard.jpg)

This is **not** how it all is supposed to work but good enough to just read some data.

# Protocol

Example [signal capture is here](https://github.com/andreyk0/libsigrokdecode-custom/blob/main/example/hpil.sr),
can be [decoded with](https://github.com/andreyk0/libsigrokdecode-custom/tree/main/decoders/hpil).

# Example output

```bash
picocom --baud 115200 --echo --logfile test.out /dev/ttyACM0
```

```
Type [C-a] [C-h] to see available commands
Terminal ready
+0.33021E+1
+0.33021E+1
+0.33021E+1
+0.33020E+1
```
