History below, if you're into that...

#### Disco TMC mk2.  Not actually TMC :)

This is a project providing a "controllable" two channel DAC on STM32 boards.
Control is via a "simple" USB Vendor class, and a tiny python module is
available that makes that easier.
Control is also available via Modbus, because.... um, why not? (well, that's the plan)

This presently builds to fit on a stm32l151c6, 32k flash, 10k ram.

[http://github.com/karlp/discotmc](discotmc) was a project at implementing
a USB TMC device (test & measurement class) with an SCPI command set, aimed
at turning popular STM32 Discovery boards into somewhat usable Arbitray
Waveform generators.

It stalled out a bit, mostly because TMC was big, usb was new to me, and the
scope was... endless. I learnt a fair bit though, and it was fun.

#### Return of disoctmc

But, parts of it were actually useful, and recently, I had a need for some
waveform generation and dusted it off.  I decided that while SCPI was interesting,
and I'm glad that [https://github.com/j123b567/scpi-parser](j123b567's scpi-parser project)
exists, it's a lot more weight than I really need, and wouldn't really fit
on some of the smaller boards I wanted to use this sort of capability on.

Hence, discotmc2.  ish.
