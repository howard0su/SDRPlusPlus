/******************************************************************************\
 * Technische Universitaet Darmstadt, Institut fuer Nachrichtentechnik
 * Copyright (c) 2001-2005
 *
 * Author(s):
 *	Volker Fischer, Andrew Murphy, Julian Cable
 *
 * Description:
 *	DRM-receiver
 * The hand over of data is done via an intermediate-buffer. The calling
 * convention is always "input-buffer, output-buffer". Additionally, the
 * DRM-parameters are fed to the function.
 *
 * 11/21/2005 Andrew Murphy, BBC Research & Development, 2005
 *	- Additions to include AMSS demodulation
 *
 ******************************************************************************
 *
 * This program is free software; you can redistribute it and/or modify it under
 * the terms of the GNU General Public License as published by the Free Software
 * Foundation; either version 2 of the License, or (at your option) any later
 * version.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
 * FOR A PARTICULAR PURPOSE. See the GNU General Public License for more
 * details.
 *
 * You should have received a copy of the GNU General Public License along with
 * this program; if not, write to the Free Software Foundation, Inc.,
 * 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA
 *
\******************************************************************************/

#include "DRMReceiver.h"

#include "util/Settings.h"
#include "util/Utilities.h"

#if 0
#include <fcd.h>
#endif

using namespace std;

const int
CDRMReceiver::MAX_UNLOCKED_COUNT = 2;

/* Implementation *************************************************************/
CDRMReceiver::CDRMReceiver(CSettings* nPsettings) : CDRMTransceiver(),
    FreqSyncAcq(),
    ChannelEstimation(),
    UtilizeFACData(), UtilizeSDCData(), MSCDemultiplexer(),
    AudioSourceDecoder(),
    RSIPacketBuf(),
    MSCDecBuf(MAX_NUM_STREAMS), MSCUseBuf(MAX_NUM_STREAMS),
    MSCSendBuf(MAX_NUM_STREAMS), iAcquRestartCnt(0),
    iAcquDetecCnt(0), iGoodSignCnt(0),
    iAudioStreamID(STREAM_ID_NOT_USED),
    iDataStreamID(STREAM_ID_NOT_USED),
    rInitResampleOffset((_REAL) 0.0),
    time_keeper(0),
    PlotManager(), iPrevSigSampleRate(0),Parameters(*(new CParameter())), pSettings(nPsettings)
{
    Parameters.SetReceiver(this);
    PlotManager.SetReceiver(this);
#ifdef HAVE_LIBGPS
    Parameters.gps_data.gps_fd = -1;
#endif

}

CDRMReceiver::~CDRMReceiver()
{
}

// void
// CDRMReceiver::SetInputDevice(string s)
// {
//     //ReceiveData.SetTrigger();
// 	ReceiveData.Stop();
// 	ReceiveData.ClearInputData();

//     /* SetSyncInput to false, can be modified by pUpstreamRSCI */
//     InputResample.SetSyncInput(false);
//     SyncUsingPil.SetSyncInput(false);
//     TimeSync.SetSyncInput(false);
//     ReceiveData.SetSoundInterface(""); // audio input
// }

// void
// CDRMReceiver::SetOutputDevice(string device)
// {
//     WriteData.SetSoundInterface(device);
//     WriteData.Init(Parameters);
// }

void
CDRMReceiver::DemodulateDRM(bool& bEnoughData)
{
    /* Resample input DRM-stream -------------------------------- */
    if (InputResample.ProcessData(Parameters, DemodDataBuf, InpResBuf))
    {
        bEnoughData = true;
    }

    /* Frequency synchronization acquisition -------------------- */
    if (FreqSyncAcq.ProcessData(Parameters, InpResBuf, FreqSyncAcqBuf))
    {
        bEnoughData = true;
    }

    /* Time synchronization ------------------------------------- */
    if (TimeSync.ProcessData(Parameters, FreqSyncAcqBuf, TimeSyncBuf))
    {
        bEnoughData = true;
        /* Use count of OFDM-symbols for detecting
         * aquisition state for acquisition detection
         * only if no signal was decoded before */
        if (Parameters.eAcquiState == AS_NO_SIGNAL)
        {
            /* Increment symbol counter and check if bound is reached */
            iAcquDetecCnt++;

            if (iAcquDetecCnt > NUM_OFDMSYM_U_ACQ_WITHOUT)
                SetInStartMode();
        }
    }

    /* OFDM-demodulation ---------------------------------------- */
    if (OFDMDemodulation.
            ProcessData(Parameters, TimeSyncBuf, OFDMDemodBuf))
    {
        bEnoughData = true;
    }

    /* Synchronization in the frequency domain (using pilots) --- */
    if (SyncUsingPil.
            ProcessData(Parameters, OFDMDemodBuf, SyncUsingPilBuf))
    {
        bEnoughData = true;
    }

    /* Channel estimation and equalisation ---------------------- */
    if (ChannelEstimation.
            ProcessData(Parameters, SyncUsingPilBuf, ChanEstBuf))
    {
        bEnoughData = true;

        /* If this module has finished, all synchronization units
           have also finished their OFDM symbol based estimates.
           Update synchronization parameters histories */
        PlotManager.UpdateParamHistories(eReceiverState);
    }

    /* Demapping of the MSC, FAC, SDC and pilots off the carriers */
    if (OFDMCellDemapping.ProcessData(Parameters, ChanEstBuf,
                                      MSCCarDemapBuf,
                                      FACCarDemapBuf, SDCCarDemapBuf))
    {
        bEnoughData = true;
    }
}

void
CDRMReceiver::DecodeDRM(bool& bEnoughData, bool& bFrameToSend)
{
    /* FAC ------------------------------------------------------ */
    if (FACMLCDecoder.ProcessData(Parameters, FACCarDemapBuf, FACDecBuf))
    {
        bEnoughData = true;
        bFrameToSend = true;
    }

    /* SDC ------------------------------------------------------ */
    if (SDCMLCDecoder.ProcessData(Parameters, SDCCarDemapBuf, SDCDecBuf))
    {
        bEnoughData = true;
    }

    /* MSC ------------------------------------------------------ */

    /* Symbol de-interleaver */
    if (SymbDeinterleaver.ProcessData(Parameters, MSCCarDemapBuf, DeintlBuf))
    {
        bEnoughData = true;
    }

    /* MLC decoder */
    if (MSCMLCDecoder.ProcessData(Parameters, DeintlBuf, MSCMLCDecBuf))
    {
        bEnoughData = true;
    }

    /* MSC demultiplexer (will leave FAC & SDC alone! */
    if (MSCDemultiplexer.ProcessData(Parameters, MSCMLCDecBuf, MSCDecBuf))
    {
        bEnoughData = true;
    }
}

void
CDRMReceiver::UtilizeDRM(bool& bEnoughData)
{
    if (UtilizeFACData.WriteData(Parameters, FACUseBuf))
    {
        bEnoughData = true;

        /* Use information of FAC CRC for detecting the acquisition
           requirement */
        DetectAcquiFAC();
    }
#if 0
        saveSDCtoFile();
#endif

    if (UtilizeSDCData.WriteData(Parameters, SDCUseBuf))
    {
        bEnoughData = true;
    }

    /* Data decoding */
    if (iDataStreamID != STREAM_ID_NOT_USED)
    {
        //cerr << "data decode processing" << endl;
        if (DataDecoder.WriteData(Parameters, MSCUseBuf[iDataStreamID]))
            bEnoughData = true;
    }

    /* Source decoding (audio) */
    if (iAudioStreamID != STREAM_ID_NOT_USED)
    {
        //cerr << "audio processing" << endl;
        if (AudioSourceDecoder.ProcessData(Parameters,
                                           MSCUseBuf[iAudioStreamID],
                                           AudSoDecBuf))
        {
            bEnoughData = true;

            /* Store the number of correctly decoded audio blocks for
             *                            the history */
            PlotManager.SetCurrentCDAud(AudioSourceDecoder.GetNumCorDecAudio());
        }
    }
    
    #if 0
    if( (iDataStreamID == STREAM_ID_NOT_USED)
       &&
         (iAudioStreamID == STREAM_ID_NOT_USED)
    ) // try and decode stream 0 as audio anyway
    {
        //cerr << "stream 0" << endl;
        if (AudioSourceDecoder.ProcessData(Parameters,
                                           MSCUseBuf[0],
                                           AudSoDecBuf))
        {
            bEnoughData = true;

            /* Store the number of correctly decoded audio blocks for
             *                            the history */
            PlotManager.SetCurrentCDAud(AudioSourceDecoder.GetNumCorDecAudio());
        }
    }
    #endif
}

void
CDRMReceiver::DetectAcquiFAC()
{
    /* Acquisition switch */
    if (!UtilizeFACData.GetCRCOk())
    {
        /* Reset "good signal" count */
        iGoodSignCnt = 0;

        iAcquRestartCnt++;

        /* Check situation when receiver must be set back in start mode */
        if ((Parameters.eAcquiState == AS_WITH_SIGNAL)
                && (iAcquRestartCnt > NUM_FAC_FRA_U_ACQ_WITH))
        {
            SetInStartMode();
        }
    }
    else
    {
        /* Set the receiver state to "with signal" not until two successive FAC
           frames are "ok", because there is only a 8-bit CRC which is not good
           for many bit errors. But it is very unlikely that we have two
           successive FAC blocks "ok" if no good signal is received */
        if (iGoodSignCnt > 0)
        {
            Parameters.eAcquiState = AS_WITH_SIGNAL;

            /* Take care of delayed tracking mode switch */
            if (iDelayedTrackModeCnt > 0)
                iDelayedTrackModeCnt--;
            else
                SetInTrackingModeDelayed();
        }
        else
        {
            /* If one CRC was correct, reset acquisition since
               we assume, that this was a correct detected signal */
            iAcquRestartCnt = 0;
            iAcquDetecCnt = 0;

            /* Set in tracking mode */
            SetInTrackingMode();

            iGoodSignCnt++;
        }
    }
}

void
CDRMReceiver::InitReceiverMode()
{
    UtilizeSDCData.GetSDCReceive()->SetSDCType(CSDCReceive::SDC_DRM);

    /* Init all modules */
    SetInStartMode();
}

void
CDRMReceiver::SetInStartMode()
{
    iUnlockedCount = MAX_UNLOCKED_COUNT;

    Parameters.Lock();
    /* Load start parameters for all modules */

    /* Define with which parameters the receiver should try to decode the
       signal. If we are correct with our assumptions, the receiver does not
       need to reinitialize */
    Parameters.InitCellMapTable(RM_ROBUSTNESS_MODE_B, SO_3);

    /* Set initial MLC parameters */
    Parameters.SetInterleaverDepth(CParameter::SI_LONG);
    Parameters.SetMSCCodingScheme(CS_3_SM);
    Parameters.SetSDCCodingScheme(CS_2_SM);

    /* Select the service we want to decode. Always zero, because we do not
       know how many services are transmitted in the signal we want to
       decode */

    /* TODO: if service 0 is not used but another service is the audio service
     * we have a problem. We should check as soon as we have information about
     * services if service 0 is really the audio service
     */

    /* Set the following parameters to zero states (initial states) --------- */
    Parameters.ResetServicesStreams();
    Parameters.ResetCurSelAudDatServ();

    /* Protection levels */
    Parameters.MSCPrLe.iPartA = 0;
    Parameters.MSCPrLe.iPartB = 1;
    Parameters.MSCPrLe.iHierarch = 0;

    /* Number of audio and data services */
    Parameters.iNumAudioService = 0;
    Parameters.iNumDataService = 0;

    /* We start with FAC ID = 0 (arbitrary) */
    Parameters.iFrameIDReceiv = 0;

    /* Set synchronization parameters */
    Parameters.rResampleOffset = rInitResampleOffset;	/* Initial resample offset */
    Parameters.rFreqOffsetAcqui = (_REAL) 0.0;
    Parameters.rFreqOffsetTrack = (_REAL) 0.0;
    Parameters.iTimingOffsTrack = 0;

    Parameters.Unlock();

    /* Initialization of the modules */
    InitsForAllModules();

    /* Activate acquisition */
    FreqSyncAcq.StartAcquisition();
    TimeSync.StartAcquisition();
    ChannelEstimation.GetTimeSyncTrack()->StopTracking();
    ChannelEstimation.StartSaRaOffAcq();
    ChannelEstimation.GetTimeWiener()->StopTracking();

    SyncUsingPil.StartAcquisition();
    SyncUsingPil.StopTrackPil();

    Parameters.Lock();
    /* Set flag that no signal is currently received */
    Parameters.eAcquiState = AS_NO_SIGNAL;

    /* Set flag for receiver state */
    eReceiverState = RS_ACQUISITION;

    /* Reset counters for acquisition decision, "good signal" and delayed
       tracking mode counter */
    iAcquRestartCnt = 0;
    iAcquDetecCnt = 0;
    iGoodSignCnt = 0;
    iDelayedTrackModeCnt = NUM_FAC_DEL_TRACK_SWITCH;

    /* Reset GUI lights */
    Parameters.ReceiveStatus.InterfaceI.SetStatus(NOT_PRESENT);
    Parameters.ReceiveStatus.InterfaceO.SetStatus(NOT_PRESENT);
    Parameters.ReceiveStatus.TSync.SetStatus(NOT_PRESENT);
    Parameters.ReceiveStatus.FSync.SetStatus(NOT_PRESENT);
    Parameters.ReceiveStatus.FAC.SetStatus(NOT_PRESENT);
    Parameters.ReceiveStatus.SDC.SetStatus(NOT_PRESENT);
    Parameters.ReceiveStatus.SLAudio.SetStatus(NOT_PRESENT);
    Parameters.ReceiveStatus.LLAudio.SetStatus(NOT_PRESENT);

    /* Clear audio decoder string */
    Parameters.audiodecoder = "";

    Parameters.Unlock();
}

void
CDRMReceiver::SetInTrackingMode()
{
    /* We do this with the flag "eReceiverState" to ensure that the following
       routines are only called once when the tracking is actually started */
    if (eReceiverState == RS_ACQUISITION)
    {
        /* In case the acquisition estimation is still in progress, stop it now
           to avoid a false estimation which could destroy synchronization */
        TimeSync.StopRMDetAcqu();

        /* Acquisition is done, deactivate it now and start tracking */
        ChannelEstimation.GetTimeWiener()->StartTracking();

        /* Reset acquisition for frame synchronization */
        SyncUsingPil.StopAcquisition();
        SyncUsingPil.StartTrackPil();

        /* Set receiver flag to tracking */
        eReceiverState = RS_TRACKING;
    }
}

void
CDRMReceiver::SetInTrackingModeDelayed()
{
    /* The timing tracking must be enabled delayed because it must wait until
       the channel estimation has initialized its estimation */
    TimeSync.StopTimingAcqu();
    ChannelEstimation.GetTimeSyncTrack()->StartTracking();
}

int
CDRMReceiver::process(int count, const dsp::complex_t* in, dsp::stereo_t* out) {
    int block_size;
    Parameters.Lock();
    block_size = Parameters.CellMappingTable.iSymbolBlockSize;
    Parameters.Unlock();

    while (count > 0) {
        CVectorEx<_REAL>* pvecOutputData = DemodDataBuf.QueryWriteBuffer();

        int process_count = std::min(count, block_size);
        pvecOutputData->Enlarge(process_count);

        // VFO outputs complex baseband centered at DC (zero IF)
        // shift by 12 kHz to get real DRM IF signal (48khz sample rate)
        const _REAL scale = 32768.0;
        const int SineTable[] = { 0, 1, 0, -1, 0 };

        for (int i = 0; i < process_count; i++) {
            iPhase = (iPhase + 1) & 3;
            _REAL rValue = in[i].re * /*COS*/ SineTable[iPhase + 1] -
                           in[i].im * /*SIN*/ SineTable[iPhase];
            (*pvecOutputData)[i] = rValue * scale;
        }
        DemodDataBuf.Put(process_count);

        Process();

        /* Play and/or save the audio */
        if (iAudioStreamID != STREAM_ID_NOT_USED) {
            // Get available audio samples from the buffer
            int availableSamples = AudSoDecBuf.GetFillLevel() / 2; // Stereo pairs (buffer contains L+R samples)
            int samplesToOutput = std::min(availableSamples, process_count);

            if (samplesToOutput > 0) {
                CVectorEx<_SAMPLE>* audioData = AudSoDecBuf.Get(samplesToOutput * 2);

                // Convert int16 stereo to normalized float stereo (-1.0 to +1.0)
                const float scale = 1.0f / 32768.0f;
                for (int i = 0; i < samplesToOutput; i++) {
                    out[i].l = (*audioData)[i * 2] * scale;
                    out[i].r = (*audioData)[i * 2 + 1] * scale;
                }
            }

            // Fill remaining samples with silence if audio buffer underrun
            for (int i = samplesToOutput; i < process_count; i++) {
                out[i].l = 0.0f;
                out[i].r = 0.0f;
            }
        }

        count -= process_count;
        in += process_count;
        out += process_count;
    }

    return count;
}

void
CDRMReceiver::Process()
{
    bool bFrameToSend = false;
    bool bEnoughData = true;

    {
        /* No I/Q recording then receive data directly in DemodDataBuf */
        // ReceiveData.ReadData(Parameters, DemodDataBuf);

        DemodulateDRM(bEnoughData);
        DecodeDRM(bEnoughData, bFrameToSend);
    }

    /* Split the data for downstream RSCI and local processing. TODO make this conditional */
    SplitFAC.ProcessData(Parameters, FACDecBuf, FACUseBuf, FACSendBuf);

    /* if we have an SDC block, make a copy and keep it until the next frame is to be sent */
    if (SDCDecBuf.GetFillLevel() == Parameters.iNumSDCBitsPerSFrame)
    {
        SplitSDC.ProcessData(Parameters, SDCDecBuf, SDCUseBuf, SDCSendBuf);
    }

    for (int i = 0; i < int(MSCDecBuf.size()); i++)
    {
        SplitMSC[i].ProcessData(Parameters, MSCDecBuf[i], MSCUseBuf[i], MSCSendBuf[i]);
    }

    /* Decoding */
    while (bEnoughData) // TODO break if stop requested
    {
        /* Init flag */
        bEnoughData = false;

        UtilizeDRM(bEnoughData);
    }

    return;
}

void
CDRMReceiver::InitsForAllModules()
{

    Parameters.bMeasureDelay = false;
    Parameters.bMeasureDoppler = false;
    Parameters.bMeasureInterference = false;
    if(Parameters.bMeasurePSDAlways)
        Parameters.bMeasurePSD = true;
    else
        Parameters.bMeasurePSD = false;

    /* Set init flags */
    SplitFAC.SetInitFlag();
    SplitSDC.SetInitFlag();
    for (size_t i = 0; i < MSCDecBuf.size(); i++)
    {
        SplitMSC[i].SetStream(i);
        SplitMSC[i].SetInitFlag();
        MSCDecBuf[i].Clear();
        MSCUseBuf[i].Clear();
        MSCSendBuf[i].Clear();
    }
    ConvertAudio.SetInitFlag();

    InputResample.SetInitFlag();
    FreqSyncAcq.SetInitFlag();
    TimeSync.SetInitFlag();
    OFDMDemodulation.SetInitFlag();
    SyncUsingPil.SetInitFlag();
    ChannelEstimation.SetInitFlag();
    OFDMCellDemapping.SetInitFlag();
    FACMLCDecoder.SetInitFlag();
    UtilizeFACData.SetInitFlag();
    SDCMLCDecoder.SetInitFlag();
    UtilizeSDCData.SetInitFlag();
    SymbDeinterleaver.SetInitFlag();
    MSCMLCDecoder.SetInitFlag();
    MSCDemultiplexer.SetInitFlag();
    AudioSourceDecoder.SetInitFlag();
    DataDecoder.SetInitFlag();

    Split.SetInitFlag();
    SplitAudio.SetInitFlag();

    SplitForIQRecord.SetInitFlag();

    /* Clear all buffers (this is especially important for the "AudSoDecBuf"
       buffer since AM mode and DRM mode use the same buffer. When init is
       called or modes are switched, the buffer could have some data left which
       lead to an overrun) */
    RecDataBuf.Clear();

    DemodDataBuf.Clear();
    IQRecordDataBuf.Clear();

    InpResBuf.Clear();
    FreqSyncAcqBuf.Clear();
    TimeSyncBuf.Clear();
    OFDMDemodBuf.Clear();
    SyncUsingPilBuf.Clear();
    ChanEstBuf.Clear();
    MSCCarDemapBuf.Clear();
    FACCarDemapBuf.Clear();
    SDCCarDemapBuf.Clear();
    DeintlBuf.Clear();
    FACDecBuf.Clear();
    SDCDecBuf.Clear();
    MSCMLCDecBuf.Clear();
    RSIPacketBuf.Clear();
    AudSoDecBuf.Clear();
    AMAudioBuf.Clear();
    AMSoEncBuf.Clear();
}

/* -----------------------------------------------------------------------------
   Initialization routines for the modules. We have to look into the modules
   and decide on which parameters the modules depend on */
void
CDRMReceiver::InitsForWaveMode()
{
    /* Reset averaging of the parameter histories (needed, e.g., because the
       number of OFDM symbols per DRM frame might have changed) */
    PlotManager.Init();

    /* After a new robustness mode was detected, give the time synchronization
       a bit more time for its job */
    iAcquDetecCnt = 0;

    /* Set init flags */
    InputResample.SetInitFlag();
    FreqSyncAcq.SetInitFlag();
    Split.SetInitFlag();

    SplitForIQRecord.SetInitFlag();

    TimeSync.SetInitFlag();
    OFDMDemodulation.SetInitFlag();
    SyncUsingPil.SetInitFlag();
    ChannelEstimation.SetInitFlag();
    OFDMCellDemapping.SetInitFlag();
    SymbDeinterleaver.SetInitFlag();	// Because of "iNumUsefMSCCellsPerFrame"
    MSCMLCDecoder.SetInitFlag();	// Because of "iNumUsefMSCCellsPerFrame"
    SDCMLCDecoder.SetInitFlag();	// Because of "iNumSDCCellsPerSFrame"
}

void
CDRMReceiver::InitsForSpectrumOccup()
{
    /* Set init flags */
    FreqSyncAcq.SetInitFlag();	// Because of bandpass filter
    OFDMDemodulation.SetInitFlag();
    SyncUsingPil.SetInitFlag();
    ChannelEstimation.SetInitFlag();
    OFDMCellDemapping.SetInitFlag();
    SymbDeinterleaver.SetInitFlag();	// Because of "iNumUsefMSCCellsPerFrame"
    MSCMLCDecoder.SetInitFlag();	// Because of "iNumUsefMSCCellsPerFrame"
    SDCMLCDecoder.SetInitFlag();	// Because of "iNumSDCCellsPerSFrame"
}

/* SDC ---------------------------------------------------------------------- */
void
CDRMReceiver::InitsForSDCCodSche()
{
    /* Set init flags */
    SDCMLCDecoder.SetInitFlag();

#ifdef USE_DD_WIENER_FILT_TIME
    ChannelEstimation.SetInitFlag();
#endif
}

void
CDRMReceiver::InitsForNoDecBitsSDC()
{
    /* Set init flag */
    SplitSDC.SetInitFlag();
    UtilizeSDCData.SetInitFlag();
}

/* MSC ---------------------------------------------------------------------- */
void
CDRMReceiver::InitsForInterlDepth()
{
    /* Can be absolutely handled seperately */
    SymbDeinterleaver.SetInitFlag();
}

void
CDRMReceiver::InitsForMSCCodSche()
{
    /* Set init flags */
    MSCMLCDecoder.SetInitFlag();
    MSCDemultiplexer.SetInitFlag();	// Not sure if really needed, look at code! TODO

#ifdef USE_DD_WIENER_FILT_TIME
    ChannelEstimation.SetInitFlag();
#endif
}

void
CDRMReceiver::InitsForMSC()
{
    /* Set init flags */
    MSCMLCDecoder.SetInitFlag();

    InitsForMSCDemux();
}

void
CDRMReceiver::InitsForMSCDemux()
{
    /* Set init flags */
    MSCDemultiplexer.SetInitFlag();
    for (size_t i = 0; i < MSCDecBuf.size(); i++)
    {
        SplitMSC[i].SetStream(i);
        SplitMSC[i].SetInitFlag();
    }
    InitsForAudParam();
    InitsForDataParam();

    /* Reset value used for the history because if an audio service was selected
       but then only a data service is selected, the value would remain with the
       last state */
    PlotManager.SetCurrentCDAud(0);
}

void
CDRMReceiver::InitsForAudParam()
{
    for (size_t i = 0; i < MSCDecBuf.size(); i++)
    {
        MSCDecBuf[i].Clear();
        MSCUseBuf[i].Clear();
        MSCSendBuf[i].Clear();
    }

    /* Set init flags */
    MSCDemultiplexer.SetInitFlag();
    int a = Parameters.GetCurSelAudioService();
    iAudioStreamID = Parameters.GetAudioParam(a).iStreamID;
    Parameters.SetNumAudioDecoderBits(Parameters.GetStreamLen(iAudioStreamID) * SIZEOF__BYTE);
    AudioSourceDecoder.SetInitFlag();
}

void
CDRMReceiver::InitsForDataParam()
{
    /* Set init flags */
    MSCDemultiplexer.SetInitFlag();
    int d = Parameters.GetCurSelDataService();
    iDataStreamID = Parameters.GetDataParam(d).iStreamID;
    Parameters.SetNumDataDecoderBits(Parameters.
                                          GetStreamLen(iDataStreamID) *
                                          SIZEOF__BYTE);
    DataDecoder.SetInitFlag();
}

void CDRMReceiver::SetFrequency(int iNewFreqkHz)
{
    Parameters.Lock();
    Parameters.SetFrequency(iNewFreqkHz);
    Parameters.Unlock();

#if 0
	{
		FCD_MODE_ENUM fme;
		unsigned int uFreq, rFreq;
		int lnbOffset = 6;
		double d = (double) (iNewFreqkHz-lnbOffset);

		//d *= 1.0 + n/1000000.0;
		uFreq = (unsigned int) d;

		fme = fcdAppSetFreq(uFreq, &rFreq);

		if ((fme != FCD_MODE_APP) || (uFreq != rFreq))
		{
			stringstream ss;
			ss << "Error in" << __FUNCTION__ << "set:" << uFreq << "read:" << rFreq;
			qDebug(ss.str().c_str()); 
		}
	}
#endif
}

/* TEST store information about alternative frequency transmitted in SDC */
void
CDRMReceiver::saveSDCtoFile()
{
    static FILE *pFile = nullptr;

    if (pFile == nullptr)
        pFile = fopen("test/altfreq.dat", "w");

    Parameters.Lock();
    size_t inum = Parameters.AltFreqSign.vecMultiplexes.size();
    for (size_t z = 0; z < inum; z++)
    {
        fprintf(pFile, "sync:%d sr:", Parameters.AltFreqSign.vecMultiplexes[z].bIsSyncMultplx);

        for (int k = 0; k < 4; k++)
            fprintf(pFile, "%d", Parameters.AltFreqSign.vecMultiplexes[z].  veciServRestrict[k]);
        fprintf(pFile, " fr:");

        for (size_t kk = 0; kk < Parameters.AltFreqSign.vecMultiplexes[z].veciFrequencies.size(); kk++)
            fprintf(pFile, "%d ", Parameters.AltFreqSign.vecMultiplexes[z].  veciFrequencies[kk]);

        fprintf(pFile, " rID:%d sID:%d   /   ",
                Parameters.AltFreqSign.vecMultiplexes[z].iRegionID,
                Parameters.AltFreqSign.vecMultiplexes[z].iScheduleID);
    }
    Parameters.Unlock();
    fprintf(pFile, "\n");
    fflush(pFile);
}

void
CDRMReceiver::LoadSettings()
{
    if (pSettings == nullptr) return;
    CSettings& s = *pSettings;

    /* Serial Number */
    string sValue = s.Get("Receiver", "serialnumber");
    if (sValue != "")
    {
        // Pad to a minimum of 6 characters
        while (sValue.length() < 6)
            sValue += "_";
        Parameters.sSerialNumber = sValue;
    }

    Parameters.GenerateReceiverID();

    /* Data files directory */
    string sDataFilesDirectory = s.Get(
        "Receiver", "datafilesdirectory", Parameters.GetDataDirectory());
    Parameters.SetDataDirectory(sDataFilesDirectory);
    s.Put("Receiver", "datafilesdirectory", Parameters.GetDataDirectory());

    /* Receiver ------------------------------------------------------------- */

    /* Sound card audio sample rate, some settings below depends on this one */
    Parameters.SetNewAudSampleRate(s.Get("Receiver", "samplerateaud", int(DEFAULT_SOUNDCRD_SAMPLE_RATE)));

    /* Sound card signal sample rate, some settings below depends on this one */
    Parameters.SetNewSigSampleRate(s.Get("Receiver", "sampleratesig", int(DEFAULT_SOUNDCRD_SAMPLE_RATE)));

    /* Signal upscale ratio */
    Parameters.SetNewSigUpscaleRatio(s.Get("Receiver", "sigupratio", int(1)));

    /* Fetch new sample rate if any */
    Parameters.FetchNewSampleRate();

    /* if 0 then only measure PSD when RSCI in use otherwise always measure it */
    Parameters.bMeasurePSDAlways = s.Get("Receiver", "measurepsdalways", 0);

    /* Channel Estimation: Frequency Interpolation */
    SetFreqInt((ETypeIntFreq)s.Get("Receiver", "freqint", int(FWIENER)));

    /* Channel Estimation: Time Interpolation */
    SetTimeInt((ETypeIntTime)s.Get("Receiver", "timeint", int(TWIENER)));

    /* Time Sync Tracking */
    SetTiSyncTracType((ETypeTiSyncTrac)s.Get("Receiver", "timesync", int(TSENERGY)));

    /* Reverberation flag */
    AudioSourceDecoder.SetReverbEffect(s.Get("Receiver", "reverb", true));

    /* Bandpass filter flag */
    FreqSyncAcq.SetRecFilter(s.Get("Receiver", "filter", false));

    /* Set parameters for frequency acquisition search window */
    const _REAL rFreqAcSeWinCenter = s.Get("command", "fracwincent", -1);
    const _REAL rFreqAcSeWinSize = s.Get("command", "fracwinsize", -1);
    FreqSyncAcq.SetSearchWindow(rFreqAcSeWinCenter, rFreqAcSeWinSize);

    /* Modified metrics flag */
    ChannelEstimation.SetIntCons(s.Get("Receiver", "modmetric", false));

    /* Number of iterations for MLC setting */
    MSCMLCDecoder.SetNumIterations(s.Get("Receiver", "mlciter", 1));

    /* Tuned Frequency */
    Parameters.SetFrequency(s.Get("Receiver", "frequency", 0));

    /* Front-end - combine into Hamlib? */
    CFrontEndParameters& FrontEndParameters = Parameters.FrontEndParameters;

    FrontEndParameters.eSMeterCorrectionType =
        CFrontEndParameters::ESMeterCorrectionType(s.Get("FrontEnd", "smetercorrectiontype", 0));

    FrontEndParameters.rSMeterBandwidth = s.Get("FrontEnd", "smeterbandwidth", (_REAL) 0.0);

    FrontEndParameters.rDefaultMeasurementBandwidth = s.Get("FrontEnd", "defaultmeasurementbandwidth", 0);

    FrontEndParameters.bAutoMeasurementBandwidth = s.Get("FrontEnd", "automeasurementbandwidth", true);

    FrontEndParameters.rCalFactorDRM = s.Get("FrontEnd", "calfactordrm", (_REAL) 0.0);

    FrontEndParameters.rCalFactorAM = s.Get("FrontEnd", "calfactoram", (_REAL) 0.0);

    FrontEndParameters.rIFCentreFreq = s.Get("FrontEnd", "ifcentrefrequency", (_REAL(DEFAULT_SOUNDCRD_SAMPLE_RATE) / 4));

    /* Latitude string (used to be just for log file) */
    double latitude, longitude;
    latitude = s.Get("GPS", "latitude", s.Get("Logfile", "latitude", (_REAL) 1000.0));
    /* Longitude string */
    longitude = s.Get("GPS", "longitude", s.Get("Logfile", "longitude", (_REAL) 1000.0));
    Parameters.Lock();
    if(-90.0 <= latitude && latitude <= 90.0 && -180.0 <= longitude  && longitude <= 180.0)
    {
        Parameters.gps_data.set = LATLON_SET;
        Parameters.gps_data.fix.latitude = latitude;
        Parameters.gps_data.fix.longitude = longitude;
    }
    else {
        Parameters.gps_data.set = 0;
    }
    bool use_gpsd = s.Get("GPS", "usegpsd", false);
    Parameters.use_gpsd=use_gpsd;
    string host = s.Get("GPS", "host", string("localhost"));
    Parameters.gps_host = host;
    Parameters.gps_port = s.Get("GPS", "port", string("2947"));
    if(use_gpsd)
        Parameters.restart_gpsd=true;

    bool permissive = s.Get("command", "permissive", false);
    Parameters.lenient_RSCI = permissive;

    Parameters.Unlock();
}

void
CDRMReceiver::SaveSettings()
{
    if (pSettings == nullptr) return;
    CSettings& s = *pSettings;

    /* Receiver ------------------------------------------------------------- */

    /* Fetch new sample rate if any */
    Parameters.FetchNewSampleRate();

    /* Sound card audio sample rate */
    s.Put("Receiver", "samplerateaud", Parameters.GetAudSampleRate());

    /* Sound card signal sample rate */
    s.Put("Receiver", "sampleratesig", Parameters.GetSoundCardSigSampleRate());

    /* Signal upscale ratio */
    s.Put("Receiver", "sigupratio", Parameters.GetSigUpscaleRatio());

    /* if 0 then only measure PSD when RSCI in use otherwise always measure it */
    s.Put("Receiver", "measurepsdalways", Parameters.bMeasurePSDAlways);

    /* Channel Estimation: Frequency Interpolation */
    s.Put("Receiver", "freqint", GetFrequencyInterpolationAlgorithm());

    /* Channel Estimation: Time Interpolation */
    s.Put("Receiver", "timeint", GetTimeInterpolationAlgorithm());

    /* Time Sync Tracking */
    s.Put("Receiver", "timesync", GetTiSyncTracType());

    /* Reverberation */
    s.Put("Receiver", "reverb", AudioSourceDecoder.GetReverbEffect());

    /* Bandpass filter flag */
    s.Put("Receiver", "filter", FreqSyncAcq.GetRecFilter());

    /* Modified metrics flag */
    s.Put("Receiver", "modmetric", ChannelEstimation.GetIntCons());

    /* Number of iterations for MLC setting */
    s.Put("Receiver", "mlciter", MSCMLCDecoder.GetInitNumIterations());

    /* Tuned Frequency */
    s.Put("Receiver", "frequency", Parameters.GetFrequency());

    /* Front-end - combine into Hamlib? */
    s.Put("FrontEnd", "smetercorrectiontype", int(Parameters.FrontEndParameters.eSMeterCorrectionType));

    s.Put("FrontEnd", "smeterbandwidth", int(Parameters.FrontEndParameters.rSMeterBandwidth));

    s.Put("FrontEnd", "defaultmeasurementbandwidth", int(Parameters.FrontEndParameters.rDefaultMeasurementBandwidth));

    s.Put("FrontEnd", "automeasurementbandwidth", Parameters.FrontEndParameters.bAutoMeasurementBandwidth);

    s.Put("FrontEnd", "calfactordrm", int(Parameters.FrontEndParameters.rCalFactorDRM));

    s.Put("FrontEnd", "calfactoram", int(Parameters.FrontEndParameters.rCalFactorAM));

    s.Put("FrontEnd", "ifcentrefrequency", int(Parameters.FrontEndParameters.rIFCentreFreq));

    /* Serial Number */
    s.Put("Receiver", "serialnumber", Parameters.sSerialNumber);

    s.Put("Receiver", "datafilesdirectory", Parameters.GetDataDirectory());

    /* GPS */
    if(Parameters.gps_data.set & LATLON_SET) {
	s.Put("GPS", "latitude", (_REAL) Parameters.gps_data.fix.latitude);
	s.Put("GPS", "longitude", (_REAL) Parameters.gps_data.fix.longitude);
    }
    s.Put("GPS", "usegpsd", Parameters.use_gpsd);
    s.Put("GPS", "host", Parameters.gps_host);
    s.Put("GPS", "port", Parameters.gps_port);
}

void CConvertAudio::InitInternal(CParameter& Parameters)
{
    iInputBlockSize = Parameters.CellMappingTable.iSymbolBlockSize;
    iOutputBlockSize = 2*iInputBlockSize;
    iMaxOutputBlockSize = 2 * int((_REAL) Parameters.GetAudSampleRate() * (_REAL) 0.4 /* 400 ms */);
}

void CConvertAudio::ProcessDataInternal(CParameter& Parameters)
{
    (void)Parameters;
    for (int i = 0; i < this->iInputBlockSize; i++)
    {
        (*this->pvecOutputData)[2*i] = _SAMPLE((*this->pvecInputData)[i]);
        (*this->pvecOutputData)[2*i+1] = _SAMPLE((*this->pvecInputData)[i]);
    }
}
