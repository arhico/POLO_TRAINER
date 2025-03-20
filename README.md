# Polo trainer
![Polo](https://github.com/arhico/POLO_TRAINER/blob/main/main/polo_s.jpg?raw=true)
another device for interacting with pets. Polo is our family friends pet; hope this device will help him to get more efficient in interactions with humans around

## Instruction
1. plug USB in
2. disk with ~3 MB capacity should appear (if not - press '0' button)
3. drop single audio file at root directory
4. press '0' button again or disconnect USB
5. touch device to play audio track

## Features status
✅ file embedding in flash + wear levelling
✅ deep sleep
✅ user runmodes: FAT disk, audio player
✅ WAV support (any samplerate, any bitdepth)
✅ MP3 support
✅ retrigger
✅ indication
🕝 automind wireless hubba protocol (automind hubba is separate project for custom wireless protocol that supports MIDI communication)

## Electronics 🕝
esp32s2 mini module. audio output is directed to DAC channels via PWM (GPIO17 & GPIO18). touch channel is defined as TOUCH_1 and it is T2 (GPIO2) right now
## Build
project based on espidf 5.4.0 with espadf for esp32s2 target
