 /*
  * Copyright (C)2007 - 2009 Motorola, Inc.
  *
  * This program is free software; you can redistribute it and/or modify
  * it under the terms of the GNU General Public License version 2 as
  * published by the Free Software Foundation.
  *
  * This program is distributed in the hope that it will be useful,
  * but WITHOUT ANY WARRANTY; without even the implied warranty of
  * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  * GNU General Public License for more details.
  *
  * You should have received a copy of the GNU General Public License
  * along with this program; if not, write to the Free Software
  * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA
  * 02111-1307, USA
  *
  */

#ifndef CPCAP_AUDIO_DRIVER_H
#define CPCAP_AUDIO_DRIVER_H

#include <linux/soundcard.h>
#include <linux/spi/cpcap-regbits.h>
#include <linux/spi/cpcap.h>

enum {
	CPCAP_AUDIO_MODE_NORMAL,	/* mode of normal audio operation */
	CPCAP_AUDIO_MODE_DAI,	/* CPCAP_AUDIO is configured for DAI testing */
	CPCAP_AUDIO_MODE_DAI_DOWNLINK = CPCAP_AUDIO_MODE_DAI,
	CPCAP_AUDIO_MODE_DAI_UPLINK,
	CPCAP_AUDIO_MODE_TTY	/* CPCAP_AUDIO is configured for TTY */
};

enum {
	CPCAP_AUDIO_CODEC_OFF,	/* codec is powered down */
	CPCAP_AUDIO_CODEC_CLOCK_ONLY, /* codec is powered down, but clocks
				   * are activated */
	CPCAP_AUDIO_CODEC_ON,	/* codec is completely operational */
	CPCAP_AUDIO_CODEC_LOOPBACK	/* xcap is put in full
				 * (analog->digital->analog) loopback mode */
};

enum {
	CPCAP_AUDIO_CODEC_RATE_8000_HZ,
		/* codec is running at 8Khz sample rate */
	CPCAP_AUDIO_CODEC_RATE_11025_HZ,
		/* codec is running at 11.025Khz sample rate */
	CPCAP_AUDIO_CODEC_RATE_12000_HZ,
		/* codec is running at 12Khz sample rate */
	CPCAP_AUDIO_CODEC_RATE_16000_HZ,
		/* codec is running at 16Khz sample rate */
	CPCAP_AUDIO_CODEC_RATE_22050_HZ,
		/* codec is running at 22.05Khz sample rate */
	CPCAP_AUDIO_CODEC_RATE_24000_HZ,
		/* codec is running at 24Khz sample rate */
	CPCAP_AUDIO_CODEC_RATE_32000_HZ,
		/* codec is running at 32Khz sample rate */
	CPCAP_AUDIO_CODEC_RATE_44100_HZ,
		/* codec is running at 44.1Khz sample rate */
	CPCAP_AUDIO_CODEC_RATE_48000_HZ,
		/* codec is running at 48Khz sample rate */
};

enum {
	CPCAP_AUDIO_CODEC_UNMUTE,	/* codec is unmuted */
	CPCAP_AUDIO_CODEC_MUTE,	/* codec is muted */
	CPCAP_AUDIO_CODEC_BYPASS_LOOP
				/* codec is bypassed
				 * (analog-only loopback mode) */
};

enum {
	CPCAP_AUDIO_STDAC_OFF,
		/* stereo dac is powered down */
	CPCAP_AUDIO_STDAC_CLOCK_ONLY,
		/* stereo dac is powered down, but clocks are activated */
	CPCAP_AUDIO_STDAC_ON
		/* stereo dac is completely operational */
};

enum {
		/* THESE MUST CORRESPOND TO XCPCAP_AUDIO SETTINGS */
	CPCAP_AUDIO_STDAC_RATE_8000_HZ,
		/* stereo dac set for 8Khz sample rate */
	CPCAP_AUDIO_STDAC_RATE_11025_HZ,
		/* stereo dac set for 11.025Khz sample rate */
	CPCAP_AUDIO_STDAC_RATE_12000_HZ,
		/* stereo dac set for 12Khz sample rate */
	CPCAP_AUDIO_STDAC_RATE_16000_HZ,
		/* stereo dac set for 16Khz sample rate */
	CPCAP_AUDIO_STDAC_RATE_22050_HZ,
		/* stereo dac set for 22.05Khz sample rate */
	CPCAP_AUDIO_STDAC_RATE_24000_HZ,
		/* stereo dac set for 24Khz sample rate */
	CPCAP_AUDIO_STDAC_RATE_32000_HZ,
		/* stereo dac set for 32Khz sample rate */
	CPCAP_AUDIO_STDAC_RATE_44100_HZ,
		/* stereo dac set for 44.1Khz sample rate */
	CPCAP_AUDIO_STDAC_RATE_48000_HZ
		/* stereo dac set for 48Khz sample rate */
};

enum {
	CPCAP_AUDIO_STDAC_UNMUTE,	/* stereo dac is unmuted */
	CPCAP_AUDIO_STDAC_MUTE	/* stereo dac is muted */
};

enum {
	CPCAP_AUDIO_ANALOG_SOURCE_OFF,
		/* Analog PGA input is disabled */
	CPCAP_AUDIO_ANALOG_SOURCE_R,
		/* Right analog PGA input is enabled */
	CPCAP_AUDIO_ANALOG_SOURCE_L,
		/* Left analog PGA input is enabled */
	CPCAP_AUDIO_ANALOG_SOURCE_STEREO
		/* Both analog PGA inputs are enabled */
};

enum {
	CPCAP_AUDIO_OUT_NONE,
		/* No audio output selected */
	CPCAP_AUDIO_OUT_HANDSET = SOUND_MASK_PHONEOUT,
		/* handset (earpiece) speaker */
	CPCAP_AUDIO_OUT_LOUDSPEAKER = SOUND_MASK_SPEAKER,
		/* loudspeaker (speakerphone) */
	CPCAP_AUDIO_OUT_LINEAR_VIBRATOR,
		/* linear vibrator, if equipped */
	CPCAP_AUDIO_OUT_MONO_HEADSET = SOUND_MASK_LINE1,
		/* mono (R channel) x.5mm headset */
	CPCAP_AUDIO_OUT_STEREO_HEADSET = SOUND_MASK_RADIO,
		/* stereo x.5mm headset */
	CPCAP_AUDIO_OUT_EXT_BUS_MONO = SOUND_MASK_CD,
		/* accessory bus mono output(EMU) */
	CPCAP_AUDIO_OUT_EMU_MONO = SOUND_MASK_LINE2,
	CPCAP_AUDIO_OUT_EXT_BUS_STEREO = SOUND_MASK_LINE3,
		/* accessory bus stereo output (EMU only) */
	CPCAP_AUDIO_OUT_EMU_STEREO = SOUND_MASK_LINE3,
	CPCAP_AUDIO_OUT_LINEOUT = SOUND_MASK_LINE,
	CPCAP_AUDIO_OUT_BT_MONO = SOUND_MASK_DIGITAL1,
	CPCAP_AUDIO_OUT_AUX_I2S = SOUND_MASK_DIGITAL2,
	CPCAP_AUDIO_OUT_NUM_OF_PATHS
		/* Max number of audio output paths */
};

enum {
	CPCAP_AUDIO_IN_NONE,
		/* No audio input selected */
	CPCAP_AUDIO_IN_HANDSET = SOUND_MASK_PHONEIN,
	CPCAP_AUDIO_IN_PRIMARY_INTERNAL = CPCAP_AUDIO_IN_HANDSET,
		/* handset (internal) microphone */
	CPCAP_AUDIO_IN_AUX_INTERNAL = SOUND_MASK_MIC,
	CPCAP_AUDIO_IN_SECONDARY_INTERNAL = CPCAP_AUDIO_IN_AUX_INTERNAL,
		/* Auxiliary (second) internal mic */
	CPCAP_AUDIO_IN_TERTIARY_INTERNAL = SOUND_MASK_LINE2,
		/* tertiary (third) internal mic */
	CPCAP_AUDIO_IN_QUATERNARY_INTERNAL = SOUND_MASK_LINE3,
		/* quaternary (fourth) internal mic */
	CPCAP_AUDIO_IN_HEADSET = SOUND_MASK_LINE1,
		/* Audio <- x.5mm headset microphone */
	CPCAP_AUDIO_IN_DUAL_EXTERNAL = SOUND_MASK_LINE,
		/* Recording from external source */
	CPCAP_AUDIO_IN_BT_MONO = SOUND_MASK_DIGITAL1,
	CPCAP_AUDIO_IN_NUM_OF_PATHS
		/* Max number of audio input paths */
};

enum {
		/* Defines the audio path type */
	CPCAP_AUDIO_AUDIO_IN_PATH,
		/* Audio input path refers to CPCAP_AUDIO_MIC_TYPE */
	CPCAP_AUDIO_AUDIO_OUT_PATH
		/* Audio output path refers to CPCAP_AUDIO_SPEAKER_TYPE */
};

enum {
	CPCAP_AUDIO_BALANCE_NEUTRAL,/* audio routed normally */
	CPCAP_AUDIO_BALANCE_R_ONLY,	/* audio routed to left channel only */
	CPCAP_AUDIO_BALANCE_L_ONLY	/* audio routed to right channel only */
};

enum {
	CPCAP_AUDIO_RAT_NONE,	/* Not in a call mode */
	CPCAP_AUDIO_RAT_UMTS,	/* 3GSM mode */
	CPCAP_AUDIO_RAT_CDMA	/* In CDMA call mode */
};

enum {
	CPCAP_AUDIO_DAI_CONFIG_NORMAL, /*stdac on 1, codec on 0, independent*/
	CPCAP_AUDIO_DAI_CONFIG_HIFI_DUAL, /*stdac on 1, codec on 0, shared PLL*/
	CPCAP_AUDIO_DAI_CONFIG_HIFI_DUPLEX_0, /*stdac and codec on 0*/
	CPCAP_AUDIO_DAI_CONFIG_HIFI_DUPLEX_1, /*stdac and codec on 1*/
};

/* Clock multipliers for A2LA register */
enum {
	CPCAP_AUDIO_A2_CLOCK_MASK  = CPCAP_BIT_A2_CLK2 | CPCAP_BIT_A2_CLK1 | CPCAP_BIT_A2_CLK0,
	CPCAP_AUDIO_A2_CLOCK_15_36 = CPCAP_BIT_A2_CLK0,
	CPCAP_AUDIO_A2_CLOCK_16_80 = CPCAP_BIT_A2_CLK1,
	CPCAP_AUDIO_A2_CLOCK_19_20 = CPCAP_BIT_A2_CLK1 | CPCAP_BIT_A2_CLK0,
	CPCAP_AUDIO_A2_CLOCK_26_00 = CPCAP_BIT_A2_CLK2,
	CPCAP_AUDIO_A2_CLOCK_33_60 = CPCAP_BIT_A2_CLK2 | CPCAP_BIT_A2_CLK0,
	CPCAP_AUDIO_A2_CLOCK_38_40 = CPCAP_BIT_A2_CLK2 | CPCAP_BIT_A2_CLK1
};

/*w21772_hdmi*/
enum {
	CPCAP_AUDIO_HDTV_ON, /*HDTV enabled*/
	CPCAP_AUDIO_HDTV_OFF
};
enum {
	CPCAP_AUDIO_MIC_UNMUTE,	/* mic is unmuted */
	CPCAP_AUDIO_MIC_MUTE,	/* mic is muted */
};

struct cpcap_audio_state {
	struct cpcap_device *cpcap;
	int mode;
	int codec_mode;
	int codec_rate;
	int codec_mute;
	int stdac_mode;
	int stdac_rate;
	int stdac_mute;
	int analog_source;
	int codec_primary_speaker;
	int codec_secondary_speaker;
	int stdac_primary_speaker;
	int stdac_secondary_speaker;
	int ext_primary_speaker;
	int ext_secondary_speaker;
	int codec_primary_balance;
	int stdac_primary_balance;
	int ext_primary_balance;
	unsigned int output_gain;
	unsigned int fm_output_gain;
	int microphone;
	unsigned int input_gain_l;
	unsigned int input_gain_r;
	int rat_type;
	int dai_config;
	int hdtv_mode;/* w21772_hdmi*/
        int mic_mute;
};

void cpcap_audio_set_audio_state(struct cpcap_audio_state *state);

void cpcap_audio_init(struct cpcap_audio_state *state);

int cpcap_audio_is_cdma_shadow(void);

#endif /* CPCAP_AUDIO_DRIVER_H */
