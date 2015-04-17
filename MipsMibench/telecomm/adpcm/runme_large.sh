#!/bin/sh
${SIMULATOR}bin/rawcaudio < data/large.pcm > output_large.adpcm
${SIMULATOR}bin/rawdaudio < data/large.adpcm > output_large.pcm
