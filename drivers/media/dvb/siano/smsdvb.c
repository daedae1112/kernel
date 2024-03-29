/****************************************************************

Siano Mobile Silicon, Inc.
MDTV receiver kernel modules.
Copyright (C) 2006-2008, Uri Shkolnik

This program is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 2 of the License, or
(at your option) any later version.

 This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program.  If not, see <http://www.gnu.org/licenses/>.

****************************************************************/

#include <linux/module.h>
#include <linux/init.h>
#include <asm/byteorder.h>

#include "dmxdev.h"
#include "dvbdev.h"
#include "dvb_demux.h"
#include "dvb_frontend.h"

#include "smscoreapi.h"
#include "smsendian.h"
#include "sms-cards.h"

DVB_DEFINE_MOD_OPT_ADAPTER_NR(adapter_nr);

struct smsdvb_client_t {
	struct list_head entry;

	struct smscore_device_t *coredev;
	struct smscore_client_t *smsclient;

	struct dvb_adapter adapter;
	struct dvb_demux demux;
	struct dmxdev dmxdev;
	struct dvb_frontend frontend;

	fe_status_t fe_status;

	struct completion tune_done;
	struct completion get_stats_done;

	unsigned long last_sample_time;

	/* todo: save freq/band instead whole struct */
	struct dvb_frontend_parameters fe_params;

	struct SMSHOSTLIB_STATISTICS_DVB_S sms_stat_dvb;
	int event_fe_state;
	int event_unc_state;
};

static struct list_head g_smsdvb_clients;
static struct mutex g_smsdvb_clientslock;


/* Events that may come from DVB v3 adapter */
static void sms_board_dvb3_event(struct smsdvb_client_t *client,
		enum SMS_DVB3_EVENTS event) {

	struct smscore_device_t *coredev = client->coredev;
	switch (event) {
	case DVB3_EVENT_INIT:
		sms_debug("DVB3_EVENT_INIT");
		sms_board_event(coredev, BOARD_EVENT_BIND);
		break;
	case DVB3_EVENT_SLEEP:
		sms_debug("DVB3_EVENT_SLEEP");
		sms_board_event(coredev, BOARD_EVENT_POWER_SUSPEND);
		break;
	case DVB3_EVENT_HOTPLUG:
		sms_debug("DVB3_EVENT_HOTPLUG");
		sms_board_event(coredev, BOARD_EVENT_POWER_INIT);
		break;
	case DVB3_EVENT_FE_LOCK:
		if (client->event_fe_state != DVB3_EVENT_FE_LOCK) {
			client->event_fe_state = DVB3_EVENT_FE_LOCK;
			sms_debug("DVB3_EVENT_FE_LOCK");
			sms_board_event(coredev, BOARD_EVENT_FE_LOCK);
		}
		break;
	case DVB3_EVENT_FE_UNLOCK:
		if (client->event_fe_state != DVB3_EVENT_FE_UNLOCK) {
			client->event_fe_state = DVB3_EVENT_FE_UNLOCK;
			sms_debug("DVB3_EVENT_FE_UNLOCK");
			sms_board_event(coredev, BOARD_EVENT_FE_UNLOCK);
		}
		break;
	case DVB3_EVENT_UNC_OK:
		if (client->event_unc_state != DVB3_EVENT_UNC_OK) {
			client->event_unc_state = DVB3_EVENT_UNC_OK;
			sms_debug("DVB3_EVENT_UNC_OK");
			sms_board_event(coredev, BOARD_EVENT_MULTIPLEX_OK);
		}
		break;
	case DVB3_EVENT_UNC_ERR:
		if (client->event_unc_state != DVB3_EVENT_UNC_ERR) {
			/*client->event_unc_state = DVB3_EVENT_UNC_ERR;*/
			sms_debug("DVB3_EVENT_UNC_ERR");
			sms_board_event(coredev, BOARD_EVENT_MULTIPLEX_ERRORS);
		}
		break;

	default:
		sms_err("Unknown dvb3 api event");
		break;
	}
}

static int smsdvb_onresponse(void *context, struct smscore_buffer_t *cb)
{
	struct smsdvb_client_t *client = (struct smsdvb_client_t *) context;
	struct SmsMsgHdr_ST *phdr = (struct SmsMsgHdr_ST *) (((u8 *) cb->p)
			+ cb->offset);
	u32 *pMsgData = (u32 *) (phdr + 1);
	/*u32 MsgDataLen = phdr->msgLength - sizeof(struct SmsMsgHdr_ST);*/
	bool is_status_update = false;

	smsendian_handle_rx_message((struct SmsMsgData_ST *) phdr);

	switch (phdr->msgType) {
	case MSG_SMS_DVBT_BDA_DATA:
		dvb_dmx_swfilter(&client->demux, (u8 *) (phdr + 1), cb->size
				- sizeof(struct SmsMsgHdr_ST));
		break;

	case MSG_SMS_ISDBT_TUNE_RES:
		sms_info("MSG_SMS_ISDBT_TUNE_RES");
	case MSG_SMS_RF_TUNE_RES:
		complete(&client->tune_done);
		break;

	case MSG_SMS_SIGNAL_DETECTED_IND:
		sms_info("MSG_SMS_SIGNAL_DETECTED_IND");
		client->sms_stat_dvb.ReceptionData.IsDemodLocked = true;
		is_status_update = true;
		break;

	case MSG_SMS_NO_SIGNAL_IND:
		sms_info("MSG_SMS_NO_SIGNAL_IND");
		client->sms_stat_dvb.ReceptionData.IsDemodLocked = false;
		is_status_update = true;
		break;

	case MSG_SMS_TRANSMISSION_IND: {
		sms_info("MSG_SMS_TRANSMISSION_IND");

		memcpy(&client->sms_stat_dvb.TransmissionData, pMsgData,
				sizeof(struct TRANSMISSION_STATISTICS_S));

		/* Mo need to correct guard interval
		 * (as opposed to old statistics message).
		 */
		CORRECT_STAT_BANDWIDTH(client->sms_stat_dvb.TransmissionData);
		CORRECT_STAT_TRANSMISSON_MODE(
				client->sms_stat_dvb.TransmissionData);
		is_status_update = true;
		break;
	}
	case MSG_SMS_HO_PER_SLICES_IND: {
		struct RECEPTION_STATISTICS_S *pReceptionData =
				&client->sms_stat_dvb.ReceptionData;
		struct SRVM_SIGNAL_STATUS_S SignalStatusData;

		/*sms_info("MSG_SMS_HO_PER_SLICES_IND");*/
		SignalStatusData.result = pMsgData[0];
		SignalStatusData.snr = pMsgData[1];
		SignalStatusData.inBandPower = (s32) pMsgData[2];
		SignalStatusData.tsPackets = pMsgData[3];
		SignalStatusData.etsPackets = pMsgData[4];
		SignalStatusData.constellation = pMsgData[5];
		SignalStatusData.hpCode = pMsgData[6];
		SignalStatusData.tpsSrvIndLP = pMsgData[7] & 0x03;
		SignalStatusData.tpsSrvIndHP = pMsgData[8] & 0x03;
		SignalStatusData.cellId = pMsgData[9] & 0xFFFF;
		SignalStatusData.reason = pMsgData[10];
		SignalStatusData.requestId = pMsgData[11];
		pReceptionData->IsRfLocked = pMsgData[16];
		pReceptionData->IsDemodLocked = pMsgData[17];
		pReceptionData->ModemState = pMsgData[12];
		pReceptionData->SNR = pMsgData[1];
		pReceptionData->BER = pMsgData[13];
		pReceptionData->RSSI = pMsgData[14];
		CORRECT_STAT_RSSI(client->sms_stat_dvb.ReceptionData);

		pReceptionData->InBandPwr = (s32) pMsgData[2];
		pReceptionData->CarrierOffset = (s32) pMsgData[15];
		pReceptionData->TotalTSPackets = pMsgData[3];
		pReceptionData->ErrorTSPackets = pMsgData[4];

		/* TS PER */
		if ((SignalStatusData.tsPackets + SignalStatusData.etsPackets)
				> 0) {
			pReceptionData->TS_PER = (SignalStatusData.etsPackets
					* 100) / (SignalStatusData.tsPackets
					+ SignalStatusData.etsPackets);
		} else {
			pReceptionData->TS_PER = 0;
		}

		pReceptionData->BERBitCount = pMsgData[18];
		pReceptionData->BERErrorCount = pMsgData[19];

		pReceptionData->MRC_SNR = pMsgData[20];
		pReceptionData->MRC_InBandPwr = pMsgData[21];
		pReceptionData->MRC_RSSI = pMsgData[22];

		is_status_update = true;
		break;
	}

	case MSG_SMS_GET_STATISTICS_EX_RES: {
		struct RECEPTION_STATISTICS_S *pReceptionData =
				&client->sms_stat_dvb.ReceptionData;
		struct SMSHOSTLIB_STATISTICS_ISDBT_S *pStatsIsdbt;
		struct SRVM_SIGNAL_STATUS_S SignalStatusData;
		/*sms_info("MSG_SMS_GET_STATISTICS_EX_RES");*/

		pMsgData++;
		pStatsIsdbt = (struct SMSHOSTLIB_STATISTICS_ISDBT_S *)pMsgData;
/*
		sms_info("%d layers detected", pStatsIsdbt->NumOfLayers);
		sms_info("InBandPwr = %d dBm", pStatsIsdbt->InBandPwr);
		sms_info("SNR = %d dB", pStatsIsdbt->SNR);
		sms_info("RSSI = %d dBm", pStatsIsdbt->RSSI);
*/

		/* update signal status */
		SignalStatusData.snr = pStatsIsdbt->SNR;
		SignalStatusData.tsPackets =
			pStatsIsdbt->LayerInfo[0].TotalTSPackets;
		SignalStatusData.etsPackets =
			pStatsIsdbt->LayerInfo[0].ErrorTSPackets;
		SignalStatusData.constellation =
			pStatsIsdbt->LayerInfo[0].Constellation;
		SignalStatusData.inBandPower = pStatsIsdbt->InBandPwr;

		/* update reception data */
		pReceptionData->IsRfLocked = pStatsIsdbt->IsRfLocked;
		pReceptionData->IsDemodLocked = pStatsIsdbt->IsDemodLocked;
		pReceptionData->ModemState = pStatsIsdbt->ModemState;
		pReceptionData->SNR = pStatsIsdbt->SNR;
		pReceptionData->BER = pStatsIsdbt->LayerInfo[0].BER;
		pReceptionData->BERErrorCount = \
			pStatsIsdbt->LayerInfo[0].BERErrorCount;
		pReceptionData->BERBitCount = \
			pStatsIsdbt->LayerInfo[0].BERBitCount;
		pReceptionData->RSSI = pStatsIsdbt->RSSI;
		CORRECT_STAT_RSSI(client->sms_stat_dvb.ReceptionData);
		pReceptionData->InBandPwr = pStatsIsdbt->InBandPwr;
		pReceptionData->CarrierOffset = pStatsIsdbt->CarrierOffset;
		pReceptionData->ErrorTSPackets = \
			pStatsIsdbt->LayerInfo[0].ErrorTSPackets;
		pReceptionData->TotalTSPackets = \
			pStatsIsdbt->LayerInfo[0].TotalTSPackets;

		/* TS PER */
		if ((SignalStatusData.tsPackets + SignalStatusData.etsPackets)
				> 0) {
			pReceptionData->TS_PER = (SignalStatusData.etsPackets
					* 100) / (SignalStatusData.tsPackets
					+ SignalStatusData.etsPackets);
		} else {
			pReceptionData->TS_PER = 0;
		}


		client->last_sample_time = jiffies_to_msecs(jiffies);
		is_status_update = true;
		complete(&client->get_stats_done);
		break;
	}

	}
	smscore_putbuffer(client->coredev, cb);

	if (is_status_update) {
		if (client->sms_stat_dvb.ReceptionData.IsDemodLocked) {
			client->fe_status = FE_HAS_SIGNAL | FE_HAS_CARRIER
			| FE_HAS_VITERBI | FE_HAS_SYNC | FE_HAS_LOCK;
			sms_board_dvb3_event(client, DVB3_EVENT_FE_LOCK);
		if (client->sms_stat_dvb.ReceptionData.ErrorTSPackets == 0)
			sms_board_dvb3_event(client, DVB3_EVENT_UNC_OK);
		else
			sms_board_dvb3_event(client, \
						DVB3_EVENT_UNC_ERR);
		} else {
			client->fe_status = 0;
			sms_board_dvb3_event(client, DVB3_EVENT_FE_UNLOCK);
		}
	}

	return 0;
}

static void smsdvb_unregister_client(struct smsdvb_client_t *client)
{
	/* must be called under clientslock */

	list_del(&client->entry);

	smscore_unregister_client(client->smsclient);
	dvb_unregister_frontend(&client->frontend);
	dvb_dmxdev_release(&client->dmxdev);
	dvb_dmx_release(&client->demux);
	dvb_unregister_adapter(&client->adapter);
	kfree(client);
}

static void smsdvb_onremove(void *context)
{
	kmutex_lock(&g_smsdvb_clientslock);

	smsdvb_unregister_client((struct smsdvb_client_t *) context);

	kmutex_unlock(&g_smsdvb_clientslock);
}

static int smsdvb_start_feed(struct dvb_demux_feed *feed)
{
	struct smsdvb_client_t *client =
		container_of(feed->demux, struct smsdvb_client_t, demux);
	struct SmsMsgData_ST PidMsg;

	sms_debug("add pid %d(%x)", feed->pid, feed->pid);

	PidMsg.xMsgHeader.msgSrcId = DVBT_BDA_CONTROL_MSG_ID;
	PidMsg.xMsgHeader.msgDstId = HIF_TASK;
	PidMsg.xMsgHeader.msgFlags = 0;
	PidMsg.xMsgHeader.msgType  = MSG_SMS_ADD_PID_FILTER_REQ;
	PidMsg.xMsgHeader.msgLength = sizeof(PidMsg);
	PidMsg.msgData[0] = feed->pid;

	smsendian_handle_tx_message((struct SmsMsgHdr_ST *)&PidMsg);
	return smsclient_sendrequest(client->smsclient, &PidMsg,
			sizeof(PidMsg));
}

static int smsdvb_stop_feed(struct dvb_demux_feed *feed)
{
	struct smsdvb_client_t *client =
		container_of(feed->demux, struct smsdvb_client_t, demux);
	struct SmsMsgData_ST PidMsg;

	sms_debug("remove pid %d(%x)", feed->pid, feed->pid);

	PidMsg.xMsgHeader.msgSrcId = DVBT_BDA_CONTROL_MSG_ID;
	PidMsg.xMsgHeader.msgDstId = HIF_TASK;
	PidMsg.xMsgHeader.msgFlags = 0;
	PidMsg.xMsgHeader.msgType  = MSG_SMS_REMOVE_PID_FILTER_REQ;
	PidMsg.xMsgHeader.msgLength = sizeof(PidMsg);
	PidMsg.msgData[0] = feed->pid;

	smsendian_handle_tx_message((struct SmsMsgHdr_ST *)&PidMsg);
	return smsclient_sendrequest(client->smsclient, &PidMsg,
			sizeof(PidMsg));
}

static int smsdvb_sendrequest_and_wait(struct smsdvb_client_t *client,
					void *buffer, size_t size,
					struct completion *completion)
{
	int rc;

	smsendian_handle_tx_message((struct SmsMsgHdr_ST *)buffer);
	rc = smsclient_sendrequest(client->smsclient, buffer, size);
	if (rc < 0)
		return rc;

	return wait_for_completion_timeout(completion, msecs_to_jiffies(2000))
			? 0 : -ETIME;
}

static int smsdvb_get_statistics_ex(struct dvb_frontend *fe)
{

	struct smsdvb_client_t *client =
	    container_of(fe, struct smsdvb_client_t, frontend);

	struct {
		struct SmsMsgHdr_ST Msg;
	} Msg;

	Msg.Msg.msgSrcId = DVBT_BDA_CONTROL_MSG_ID;
	Msg.Msg.msgDstId = HIF_TASK;
	Msg.Msg.msgFlags = 0;
	Msg.Msg.msgType = MSG_SMS_GET_STATISTICS_EX_REQ;
	Msg.Msg.msgLength = sizeof(Msg);

	smsendian_handle_tx_message((struct SmsMsgHdr_ST *)&Msg);
	return smsdvb_sendrequest_and_wait(client, &Msg, sizeof(Msg),
					   &client->get_stats_done);

}

static int smsdvb_update_stats(struct smsdvb_client_t *client,
		struct dvb_frontend *fe) {
	int rc = 0;
	unsigned long time_now = jiffies_to_msecs(jiffies);

	/*
	 * warning : do not remove this operation mode check
	 * smsdvb_get_statistics_ex will fail on DVBT
	 */
	if  (smscore_get_device_mode(client->coredev) == DEVICE_MODE_DVBT_BDA)
		return rc;

	if ((!client->last_sample_time) ||
		(time_now - client->last_sample_time > 100)) {
		/*sms_debug("%lu ms since last sample time, getting statistics",
				(time_now - client->last_sample_time));*/
		rc = smsdvb_get_statistics_ex(fe);
		if (rc < 0) {
			sms_err("smsdvb_get_statistics_ex, rc = %d", rc);
			return rc;
		}
	}

	return rc;
}

static int smsdvb_read_status(struct dvb_frontend *fe, fe_status_t *stat)
{
	struct smsdvb_client_t *client;
	client = container_of(fe, struct smsdvb_client_t, frontend);

	int rc = smsdvb_update_stats(client, fe);
	if (rc < 0)
		return rc;

	*stat = client->fe_status;

	return 0;
}

static int smsdvb_read_ber(struct dvb_frontend *fe, u32 *ber)
{
	struct smsdvb_client_t *client;
	client = container_of(fe, struct smsdvb_client_t, frontend);

	int rc = smsdvb_update_stats(client, fe);
	if (rc < 0)
		return rc;

	*ber = client->sms_stat_dvb.ReceptionData.BER;

	return 0;
}

static int smsdvb_read_signal_strength(struct dvb_frontend *fe, u16 *strength)
{
	struct smsdvb_client_t *client;
	client = container_of(fe, struct smsdvb_client_t, frontend);

	int rc = smsdvb_update_stats(client, fe);
	if (rc < 0)
		return rc;

	if (client->sms_stat_dvb.ReceptionData.InBandPwr < -95)
		*strength = 0;
		else if (client->sms_stat_dvb.ReceptionData.InBandPwr > -29)
			*strength = 100;
		else
			*strength =
				(client->sms_stat_dvb.ReceptionData.InBandPwr
				+ 95) * 3 / 2;

	return 0;
}

static int smsdvb_read_snr(struct dvb_frontend *fe, u16 *snr)
{
	struct smsdvb_client_t *client;
	client = container_of(fe, struct smsdvb_client_t, frontend);

	int rc = smsdvb_update_stats(client, fe);
	if (rc < 0)
		return rc;

	*snr = client->sms_stat_dvb.ReceptionData.SNR;

	return 0;
}

static int smsdvb_read_ucblocks(struct dvb_frontend *fe, u32 *ucblocks)
{
	struct smsdvb_client_t *client;
	client = container_of(fe, struct smsdvb_client_t, frontend);

	int rc = smsdvb_update_stats(client, fe);
	if (rc < 0)
		return rc;
	*ucblocks = client->sms_stat_dvb.ReceptionData.ErrorTSPackets;

	return 0;
}

static int smsdvb_get_tune_settings(struct dvb_frontend *fe,
				    struct dvb_frontend_tune_settings *tune)
{
	sms_debug("");

	tune->min_delay_ms = 400;
	tune->step_size = 250000;
	tune->max_drift = 0;
	return 0;
}

static int smsdvb_tune_isdbt(struct smsdvb_client_t *client,
			       struct dvb_frontend_parameters *fep)
{
	struct {
		struct SmsMsgHdr_ST Msg;
		u32 Data[4];
	} Msg;

	Msg.Msg.msgSrcId = DVBT_BDA_CONTROL_MSG_ID;
	Msg.Msg.msgDstId = HIF_TASK;
	Msg.Msg.msgFlags = 0;
	Msg.Msg.msgType = MSG_SMS_ISDBT_TUNE_REQ;
	Msg.Msg.msgLength = sizeof(Msg);
	Msg.Data[0] = fep->frequency;
	Msg.Data[1] = BW_ISDBT_1SEG;
	Msg.Data[2] = 12000000;
	Msg.Data[3] = 0;

	sms_debug("freq %d msgType %d Msg.Data[1] %d",
		  fep->frequency, Msg.Msg.msgType, BW_ISDBT_1SEG);

	return smsdvb_sendrequest_and_wait(client, &Msg, sizeof(Msg),
					   &client->tune_done);
}

static int smsdvb_tune_dvbt(struct smsdvb_client_t *client,
			     struct dvb_frontend_parameters *fep)
{
	struct {
		struct SmsMsgHdr_ST	Msg;
		u32		Data[3];
	} Msg;

	Msg.Msg.msgSrcId  = DVBT_BDA_CONTROL_MSG_ID;
	Msg.Msg.msgDstId  = HIF_TASK;
	Msg.Msg.msgFlags  = 0;
	Msg.Msg.msgType   = MSG_SMS_RF_TUNE_REQ;
	Msg.Msg.msgLength = sizeof(Msg);
	Msg.Data[0] = fep->frequency;
	Msg.Data[2] = 12000000;

	sms_debug("freq %d band %d",
		  fep->frequency, fep->u.ofdm.bandwidth);

	switch (fep->u.ofdm.bandwidth) {
	case BANDWIDTH_8_MHZ:
		Msg.Data[1] = BW_8_MHZ;
		break;
	case BANDWIDTH_7_MHZ:
		Msg.Data[1] = BW_7_MHZ;
		break;
	case BANDWIDTH_6_MHZ:
		Msg.Data[1] = BW_6_MHZ;
		break;
/*
	case BANDWIDTH_5_MHZ:
		Msg.Data[1] = BW_5_MHZ;
		break;
*/
	case BANDWIDTH_AUTO:
		return -EOPNOTSUPP;
	default:
		return -EINVAL;
	}

	return smsdvb_sendrequest_and_wait(client, &Msg, sizeof(Msg),
					   &client->tune_done);
}

static int smsdvb_set_frontend(struct dvb_frontend *fe,
			       struct dvb_frontend_parameters *fep)
{
	struct smsdvb_client_t *client =
	    container_of(fe, struct smsdvb_client_t, frontend);

	client->fe_status = FE_HAS_SIGNAL;
	client->event_fe_state = -1;
	client->event_unc_state = -1;

	switch (client->coredev->mode) {
	case DEVICE_MODE_DVBT_BDA:
		return smsdvb_tune_dvbt(client, fep);
	case DEVICE_MODE_ISDBT_BDA:
		return smsdvb_tune_isdbt(client, fep);
	default:
		sms_err("SMS Device mode %d is not set for DVB operation.", \
			client->coredev->mode);
		return -EINVAL;
	}
}

static int smsdvb_get_frontend(struct dvb_frontend *fe,
			       struct dvb_frontend_parameters *fep)
{
	struct smsdvb_client_t *client =
		container_of(fe, struct smsdvb_client_t, frontend);

	sms_debug("");

	/* todo: - need to update fe_params */
	memcpy(fep, &client->fe_params,
	       sizeof(struct dvb_frontend_parameters));

	return 0;
}

static int smsdvb_init(struct dvb_frontend *fe)
{
	struct smsdvb_client_t *client =
		container_of(fe, struct smsdvb_client_t, frontend);

	sms_board_dvb3_event(client, DVB3_EVENT_INIT);
	return 0;
}

static int smsdvb_sleep(struct dvb_frontend *fe)
{
	struct smsdvb_client_t *client =
		container_of(fe, struct smsdvb_client_t, frontend);

	sms_board_dvb3_event(client, DVB3_EVENT_SLEEP);

	return 0;
}

static void smsdvb_release(struct dvb_frontend *fe)
{
	/* do nothing */
}

static struct dvb_frontend_ops smsdvb_fe_ops = {
	.info = {
		 .name = "Siano Mobile Digital MDTV Receiver",
		.type			= FE_OFDM,
		 .frequency_min = 164000000,
		.frequency_max		= 867250000,
		.frequency_stepsize	= 250000,
		.caps = FE_CAN_INVERSION_AUTO |
			FE_CAN_FEC_1_2 | FE_CAN_FEC_2_3 | FE_CAN_FEC_3_4 |
			FE_CAN_FEC_5_6 | FE_CAN_FEC_7_8 | FE_CAN_FEC_AUTO |
			FE_CAN_QPSK | FE_CAN_QAM_16 | FE_CAN_QAM_64 |
			FE_CAN_QAM_AUTO | FE_CAN_TRANSMISSION_MODE_AUTO |
			FE_CAN_GUARD_INTERVAL_AUTO |
		 FE_CAN_RECOVER | FE_CAN_HIERARCHY_AUTO,
	},

	.release = smsdvb_release,

	.set_frontend = smsdvb_set_frontend,
	.get_frontend = smsdvb_get_frontend,
	.get_tune_settings = smsdvb_get_tune_settings,

	.read_status = smsdvb_read_status,
	.read_ber = smsdvb_read_ber,
	.read_signal_strength = smsdvb_read_signal_strength,
	.read_snr = smsdvb_read_snr,
	.read_ucblocks = smsdvb_read_ucblocks,

	.init = smsdvb_init,
	.sleep = smsdvb_sleep,
};

static int smsdvb_hotplug(struct smscore_device_t *coredev,
			  struct device *device, int arrival)
{
	struct smsclient_params_t params;
	struct smsdvb_client_t *client;
	int rc;
	int mode = smscore_get_device_mode(coredev);

	/* device removal handled by onremove callback */
	if (!arrival)
		return 0;

	if ((mode != DEVICE_MODE_DVBT_BDA) &&
	     (mode != DEVICE_MODE_ISDBT_BDA)) {
		sms_err("SMS Device mode is not set for "
			"DVB operation.");
		return 0;
	}

	client = kzalloc(sizeof(struct smsdvb_client_t), GFP_KERNEL);
	if (!client) {
		sms_err("kmalloc() failed");
		return -ENOMEM;
	}

	/* register dvb adapter */
#ifdef SMS_DVB_OLD_DVB_REGISTER_ADAPTER
	rc = dvb_register_adapter(&client->adapter,
				  sms_get_board(smscore_get_board_id(coredev))->
				  name, THIS_MODULE, device);
#else
	rc = dvb_register_adapter(&client->adapter,
				  sms_get_board(smscore_get_board_id(coredev))->
				  name, THIS_MODULE, device, adapter_nr);
#endif
	if (rc < 0) {
		sms_err("dvb_register_adapter() failed %d", rc);
		goto adapter_error;
	}

	/* init dvb demux */
	client->demux.dmx.capabilities = DMX_TS_FILTERING;
	client->demux.filternum = 32; /* todo: nova ??? */
	client->demux.feednum = 32;
	client->demux.start_feed = smsdvb_start_feed;
	client->demux.stop_feed = smsdvb_stop_feed;

	rc = dvb_dmx_init(&client->demux);
	if (rc < 0) {
		sms_err("dvb_dmx_init failed %d", rc);
		goto dvbdmx_error;
	}

	/* init dmxdev */
	client->dmxdev.filternum = 32;
	client->dmxdev.demux = &client->demux.dmx;
	client->dmxdev.capabilities = 0;

	rc = dvb_dmxdev_init(&client->dmxdev, &client->adapter);
	if (rc < 0) {
		sms_err("dvb_dmxdev_init failed %d", rc);
		goto dmxdev_error;
	}

	/* init and register frontend */
	memcpy(&client->frontend.ops, &smsdvb_fe_ops,
	       sizeof(struct dvb_frontend_ops));

	rc = dvb_register_frontend(&client->adapter, &client->frontend);
	if (rc < 0) {
		sms_err("frontend registration failed %d", rc);
		goto frontend_error;
	}

	params.initial_id = 1;
	params.data_type = MSG_SMS_DVBT_BDA_DATA;
	params.onresponse_handler = smsdvb_onresponse;
	params.onremove_handler = smsdvb_onremove;
	params.context = client;

	rc = smscore_register_client(coredev, &params, &client->smsclient);
	if (rc < 0) {
		sms_err("smscore_register_client() failed %d", rc);
		goto client_error;
	}

	client->coredev = coredev;

	init_completion(&client->tune_done);
	init_completion(&client->get_stats_done);

	kmutex_lock(&g_smsdvb_clientslock);

	list_add(&client->entry, &g_smsdvb_clients);

	kmutex_unlock(&g_smsdvb_clientslock);

	client->event_fe_state = -1;
	client->event_unc_state = -1;
	sms_board_dvb3_event(client, DVB3_EVENT_HOTPLUG);

	sms_info("success");
	return 0;

client_error:
	dvb_unregister_frontend(&client->frontend);

frontend_error:
	dvb_dmxdev_release(&client->dmxdev);

dmxdev_error:
	dvb_dmx_release(&client->demux);

dvbdmx_error:
	dvb_unregister_adapter(&client->adapter);

adapter_error:
	kfree(client);
	return rc;
}

int smsdvb_register(void)
{
	int rc;

	INIT_LIST_HEAD(&g_smsdvb_clients);
	kmutex_init(&g_smsdvb_clientslock);

	rc = smscore_register_hotplug(smsdvb_hotplug);

	sms_debug("");

	return rc;
}

void smsdvb_unregister(void)
{
	smscore_unregister_hotplug(smsdvb_hotplug);

	kmutex_lock(&g_smsdvb_clientslock);

	while (!list_empty(&g_smsdvb_clients))
		smsdvb_unregister_client((struct smsdvb_client_t *)
					 g_smsdvb_clients.next);

	kmutex_unlock(&g_smsdvb_clientslock);
}

MODULE_DESCRIPTION("SMS DVB subsystem adaptation module");
MODULE_AUTHOR("Siano Mobile Silicon, Inc. (uris@siano-ms.com)");
MODULE_LICENSE("GPL");
