
#include <stdint.h>
#include <stdio.h>
#include <libopencm3/stm32/dac.h>
#include <libopencm3/stm32/dma.h>

#include "funcgen.h"



/* FIXME - move this to a layer of platform code, with "null", "f4" and "l1" platforms */
void funcgen_plat_dma_setup(int channel, const uint16_t *wave_table, int wave_table_count) {
	/* DAC channel 1 (tim6) uses DMA channel 2. */
	/* DAC channel 2 (tim7) uses DMA channel 3. */
	uint8_t dchan;
	uint32_t daddr;
	switch (channel) {
	case CHANNEL_2:
		dchan = DMA_CHANNEL3;
		daddr = (uint32_t) & DAC_DHR12R2;
		break;
	default:
	case CHANNEL_1:
		dchan = DMA_CHANNEL2;
		daddr = (uint32_t) & DAC_DHR12R1;
		break;
	}
#define WANT_USE_DMA_INTERRUPTS 0
#if WANT_USE_DMA_INTERRUPTS
	UNFIXED :)
	nvic_enable_irq(NVIC_DMA1_STREAM5_IRQ);
	dma_enable_transfer_complete_interrupt(DMA1, DMA_STREAM5);
#endif

	dma_channel_reset(DMA1, dchan);
	printf("setting DMA (%u) from %#x of len: %d\n", dchan, (unsigned int)wave_table, wave_table_count);
	dma_set_memory_address(DMA1, dchan, (uint32_t) wave_table);
	dma_set_memory_size(DMA1, dchan, DMA_CCR_MSIZE_16BIT);
	dma_enable_memory_increment_mode(DMA1, dchan);
	dma_set_read_from_memory(DMA1, dchan);
	dma_set_peripheral_address(DMA1, dchan, daddr);
	dma_set_peripheral_size(DMA1, dchan, DMA_CCR_PSIZE_16BIT);
	dma_set_number_of_data(DMA1, dchan, wave_table_count);
	dma_enable_circular_mode(DMA1, dchan);
	dma_enable_channel(DMA1, dchan);

}

