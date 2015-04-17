#!/bin/sh
${SIMULATOR}bin/rawcaudio < data/small.pcm > output_small.adpcm
${SIMULATOR}bin/rawdaudio < data/small.adpcm > output_small.pcm
