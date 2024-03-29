#include <stdint.h>
#include <stdio.h>
#include <math.h>

#include "funcgen.h"

static struct funcgen_output_t _output1 = {
	.enabled = false,
	.ampl = 0.1,
	.offset = FULL_SCALE / 2,
	.freq = 1000,
};

static struct funcgen_output_t _output2 = {
	.enabled = false,
	.ampl = 0.1,
	.offset = FULL_SCALE / 2,
	.freq = 1000,
};

static struct funcgen_state_t state = {
	.outputs = { &_output1, &_output2 },
};

/* http://yehar.com/blog/?p=1220 perhaps */
/* From ST's example code */
const uint16_t lut_sine[] = {
	2047, 2447, 2831, 3185, 3498, 3750, 3939, 4056, 4095, 4056,
	3939, 3750, 3495, 3185, 2831, 2447, 2047, 1647, 1263, 909,
	599, 344, 155, 38, 0, 38, 155, 344, 599, 909, 1263, 1647, 2047
};

// TODO - need to extract common ch1/ch2 code...


/*
 * I ARE DUM
 * http://stackoverflow.com/questions/3304513/stretching-out-an-array
 */
static float interp1(float x, const uint16_t a[], int n)
{
	if (x <= 0) {
		return a[0];
	}
	if (x >= n - 1) {
		return a[n - 1];
	}
	int j = x;
	return a[j] + (x - j) * (a[j + 1] - a[j]);
}

static void calculate_output(const uint16_t *source, int source_len,
	uint16_t *dest, int dest_len,
	float ampl, float offset)
{
	float nom = source_len - 1;
	float denom = dest_len - 1;
	float step = nom / denom;
	float offset_bits = (offset / FULL_SCALE) * 4095;
	for (int i = 0; i < dest_len; i++) {
		float si = interp1(i*step, source, source_len);
		dest[i] = si * (ampl / FULL_SCALE) + offset_bits;
	}
}

void funcgen_sin(int channel, float frequency, float ampl, float offset) {
	uint16_t *wavedata = state.outputs[channel]->waveform;
	int dest_len = ARRAY_LENGTH(state.outputs[channel]->waveform);
	
	/* Take the input wave and calculate the wavetable for DMA */
	calculate_output(lut_sine, ARRAY_LENGTH(lut_sine), wavedata, dest_len, ampl, offset);
	//calculate_output(lut_sine, 32, wavedata, FUNCGEN_WAVE_SIZE, ampl, offset);

	float usecs_per_wave = 1000000 / frequency;
	int nanos_per_sample = ceilf(1000 * usecs_per_wave / (dest_len * 1.0f));
	printf("Requested freq: %f, usecs/wave: %f, timer nsecs: %d\n", frequency, usecs_per_wave, nanos_per_sample);
	
	/*+++ hardware setup +++*/
	funcgen_plat_timer_setup(channel, nanos_per_sample);
	funcgen_plat_dma_setup(channel, wavedata, dest_len);
	funcgen_plat_dac_setup(channel, true);
	/*++++++++++++++++++++++*/
	
        state.outputs[channel]->mode = OUTPUT_MODE_SINE;
	state.outputs[channel]->enabled = true;
	state.outputs[channel]->freq = frequency;
	state.outputs[channel]->ampl = ampl;
	state.outputs[channel]->offset = offset;
	/* we're not doing any tricks on variable lengths for better frequency control at the moment */
	state.outputs[channel]->waveform_length = dest_len;
}

void funcgen_triangle(int channel, float frequency, float ampl, float offset)
{
	float usecs_per_wave = 1000000 / frequency;
	int nanos_per_sample = 1000 * usecs_per_wave;

	/* You _can_ do triangle much like sine, but we're going
	 * to defer to the platform's built in triangle generation, if
	 * for no other reason than to try the feature out. */
	/* So.... this isn't very useful, the way it works means the
	 * period is dependent on the amplitude chosen */
	funcgen_plat_timer_setup(channel, usecs_per_wave);
	funcgen_plat_dac_setup(channel, false);

	uint16_t amp16 = ampl / FULL_SCALE * 0xf;
	uint16_t off16 = offset / FULL_SCALE * 4095;
	printf("Requested triangle: ampls: %x, off16: %x, timer nsecs: %d\n", amp16, off16, nanos_per_sample);
	funcgen_plat_dac_setup_triangle(channel, amp16, off16);

	state.outputs[channel]->mode = OUTPUT_MODE_TRIANGLE;
	state.outputs[channel]->enabled = true;
	state.outputs[channel]->freq = frequency;
	state.outputs[channel]->ampl = ampl;
	state.outputs[channel]->offset = offset;
}

void funcgen_prepare_udata(int channel, int data_len)
{
	state.outputs[channel]->waveform_length = data_len;
}

void funcgen_set_udata(int channel, int offset, uint16_t *data, int len)
{
	uint16_t *dst = &state.outputs[channel]->waveform[offset];
	// no need for freq, that comes in the other channel, set udata is jsut for filling the buffer (need both expected, offset, and remaining, anddddd freq!)
	// but need to carefully fix length of full sample set
	for (int i = 0; i < len; i++) {
		dst[i] = data[i];
	}
}

void funcgen_user(int channel, float frequency, float ampl, float offset) {
        uint16_t *wavedata = state.outputs[channel]->waveform;
	int dest_len = state.outputs[channel]->waveform_length;

	/*
	 * user is actually simpler. we have a sample count, and a target freq.
	 * we therefore have 1/f / n seconds per sample.
	 */

	float usecs_per_wave = 1000000 / frequency;
	int nanos_per_sample = ceil(1000 * usecs_per_wave / (dest_len * 1.0));
	printf("Requested freq: %f, usecs/wave: %f, timer nsecs: %d\n", frequency, usecs_per_wave, nanos_per_sample);

	/*+++ hardware setup +++*/
	funcgen_plat_timer_setup(channel, nanos_per_sample);
	funcgen_plat_dma_setup(channel, wavedata, dest_len);
	funcgen_plat_dac_setup(channel, true);
	/*++++++++++++++++++++++*/

        state.outputs[channel]->mode = OUTPUT_MODE_USER;
	state.outputs[channel]->enabled = true;
	state.outputs[channel]->freq = frequency;
	state.outputs[channel]->ampl = ampl;
	state.outputs[channel]->offset = offset;
	/* we're not doing any tricks on variable lengths for better frequency control at the moment */
	state.outputs[channel]->waveform_length = dest_len;
}

void funcgen_output(int channel, bool enable)
{
	printf("setting channel %d to %s\n", channel, enable ? "ON" : "off");
	funcgen_plat_output(channel, enable);
	if (channel == 2) {
		state.outputs[0]->enabled = enable;
		state.outputs[1]->enabled = enable;
	} else {
		state.outputs[channel]->enabled = enable;
	}
}

void funcgen_buffer(int channel, bool enable)
{
	funcgen_plat_buffer(channel, enable);
	// FIXME should be in the state object too really.
}

void funcgen_sync(void)
{
	/*
	 * leave the timers running, but disconnect the triggers while you reset
	 * the waveforms.
	 */
	funcgen_output(2, false);
	for (int channel = 0; channel < 2; channel++) {
		funcgen_output(channel, false);
		uint16_t *wavedata = state.outputs[channel]->waveform;
		int dest_len = state.outputs[channel]->waveform_length;
		funcgen_plat_dma_setup(channel, wavedata, dest_len);
		funcgen_plat_dma_setup(channel, wavedata, dest_len);
	}
	/* In theory, if the timers were at different points, they would still
	 * be somewhat out of sync. (one will trigger a dma request before
	 * the other) In reality, this doesn't seem to matter,
	 * but perhaps just not good enough measurement equipment.
	funcgen_plat_timer_reset(0);
	funcgen_plat_timer_reset(1);
	*/
	funcgen_output(2, true);
}


struct funcgen_state_t* funcgen_getstate(void) {
	return &state;
}