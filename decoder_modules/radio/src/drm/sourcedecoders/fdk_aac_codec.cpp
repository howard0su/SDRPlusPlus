/******************************************************************************\
 *
 * Copyright (c) 2013
 *
 * Author(s):
 *  David Flamand
 *
 * Description:
 *  AAC codec class
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

#include "fdk_aac_codec.h"
#include <fdk-aac/aacdecoder_lib.h>
#include <fdk-aac/FDK_audio.h>
#include "SDC/SDC.h"
#include <cstring>
#include <utils/flog.h>

using namespace std;

FdkAacCodec::FdkAacCodec() :
    hDecoder(nullptr), bUsac(false)
{
    decode_buf.resize(1024 * 1024);
}

FdkAacCodec::~FdkAacCodec()
{
	DecClose();
}

static void aacinfo(LIB_INFO& inf) {
    LIB_INFO info[FDK_MODULE_LAST];
    FDKinitLibInfo(info);
    aacDecoder_GetLibInfo(info);
    stringstream version;
    for (int i = 0; info[i].module_id != FDK_NONE && i < FDK_MODULE_LAST; i++) {
        if(info[i].module_id == FDK_AACDEC) {
            inf = info[i];
        }
    }
}

string
FdkAacCodec::DecGetVersion()
{
    stringstream version;
    LIB_INFO info;
    aacinfo(info);
    version << info.title << " version " << info.versionStr << " " << info.build_date;
    return version.str();
}

bool
FdkAacCodec::CanDecode(CAudioParam::EAudCod eAudioCoding)
{
    LIB_INFO linfo;
    aacinfo(linfo);

    if(eAudioCoding == CAudioParam::AC_AAC) {
        if((linfo.flags & CAPF_AAC_DRM_BSFORMAT) == 0) {
            flog::error("FDK aac but codec has no drm");
            return false;
        }
        if((linfo.flags & CAPF_SBR_DRM_BS) ==0 ) {
            flog::error("FDK aac but codec has no sbr");
            return false;
        }
        if((linfo.flags & CAPF_SBR_PS_DRM) == 0) {
            flog::error("FDK aac but codec has no parametric stereo");
            return false;
        }
        return true;
    }

#ifdef HAVE_USAC
    if (eAudioCoding == CAudioParam::AC_xHE_AAC) {
        if ((linfo.flags & CAPF_AAC_USAC) != 0) {
            return true;
        }
        flog::error("FDK xHE-AAC requested but codec built without USAC support");
    }
#else
    if (eAudioCoding == CAudioParam::AC_xHE_AAC) {
        flog::error("FDK xHE-AAC requested but codec built without USAC support");
    }
#endif

    //printf("FDK can decode NO\n");
    return false;
}

static void logAOT(std::ostringstream& oss, const CStreamInfo& info) {
    switch (info.aot) {
    case AUDIO_OBJECT_TYPE::AOT_DRM_AAC:
        oss << " AAC";
        break;
    case AUDIO_OBJECT_TYPE::AOT_DRM_SBR:
        oss << " AAC+SBR";
        break;
    case AUDIO_OBJECT_TYPE::AOT_DRM_MPEG_PS:
        oss << " AAC+SBR+PS";
        break;
    case AUDIO_OBJECT_TYPE::AOT_DRM_SURROUND:
        oss << " AAC+Surround";
        break;
#ifdef HAVE_USAC
    case AUDIO_OBJECT_TYPE::AOT_USAC:
    case AUDIO_OBJECT_TYPE::AOT_DRM_USAC:
        oss << " xHE-AAC";
        break;
#endif
    default:
        oss << "unknown object type";
    }
    if(info.extAot == AUDIO_OBJECT_TYPE::AOT_SBR) {
        oss << "+SBR";
    }
}
    /*
     * AC_ER_VCB11 1 means use  virtual codebooks
     * AC_ER_RVLC  1 means use huffman codeword reordering
     * AC_ER_HCR   1 means use virtual codebooks
     * AC_SCALABLE AAC Scalable
     * AC_ELD AAC-ELD
     * AC_LD AAC-LD
     * AC_ER ER syntax
     * AC_BSAC BSAC
     * AC_USAC USAC
     * AC_RSV603DA RSVD60 3D audio
     * AC_HDAAC HD-AAC
     * AC_RSVD50 Rsvd50
     * AC_SBR_PRESENT SBR present flag (from ASC)
     * AC_SBRCRC  SBR CRC present flag. Only relevant for AAC-ELD for now.
     * AC_PS_PRESENT PS present flag (from ASC or implicit)
     * AC_MPS_PRESENT   MPS present flag (from ASC or implicit)
     * AC_DRM DRM bit stream syntax
     * AC_INDEP Independency flag
     * AC_MPEGD_RES MPEG-D residual individual channel data.
     * AC_SAOC_PRESENT SAOC Present Flag
     * AC_DAB DAB bit stream syntax
     * AC_ELD_DOWNSCALE ELD Downscaled playout
     * AC_LD_MPS Low Delay MPS.
     * AC_DRC_PRESENT  Dynamic Range Control (DRC) data found.
     * AC_USAC_SCFGI3  USAC flag: If stereoConfigIndex is 3 the flag is set.
     */

static void logFlags(std::ostringstream& oss, const CStreamInfo& info) {

    if((info.flags & AC_USAC) == AC_USAC) {
        oss << "+USAC";
    }
    if((info.flags & AC_SBR_PRESENT) == AC_SBR_PRESENT) {
        oss << "+SBR";
    }
    if((info.flags & AC_SBRCRC) == AC_SBRCRC) {
        oss << "+SBR-CRC";
    }
    if((info.flags & AC_PS_PRESENT) == AC_PS_PRESENT) {
        oss << "+PS";
    }
    if((info.flags & AC_MPS_PRESENT) == AC_MPS_PRESENT) {
        oss << "+MPS";
    }
    if((info.flags & AC_INDEP) == AC_INDEP) {
        oss << " (independent)";
    }
}

#if 1
static void logNumbers(std::ostringstream& oss, const CStreamInfo& info) {
    oss << " channels_coded=" << info.aacNumChannels
         << " coded_sample_rate=" << info.aacSampleRate
         << " channels=" << info.numChannels
         << " channel_config=" << info.channelConfig
         << " sample_rate=" << info.sampleRate
         << " extended_sample_rate=" << info.extSamplingRate
         << " samples_per_frame=" << info.aacSamplesPerFrame
         << " decoded_audio_frame_size=" << info.frameSize
         << " flags=" << hex << info.flags << dec;
    oss << " channel_0_type=" << int(info.pChannelType[0]) << " index=" << int(info.pChannelIndices[0]);
    if(info.numChannels==2)
        oss << " channel_1_type=" << int(info.pChannelType[1]) << " index=" << int(info.pChannelIndices[1]);
}
#endif

/*      FROM AACCodec:
    In case of SBR, AAC sample rate is half the total sample rate. Length of output is doubled if SBR is used
    if (AudioParam.eSBRFlag == CAudioParam::SB_USED)
    {
        iAudioSampleRate = iAACSampleRate * 2;
    }
    else
    {
        iAudioSampleRate = iAACSampleRate;
    }
*/

bool
FdkAacCodec::DecOpen(const CAudioParam& AudioParam, int& iAudioSampleRate)
{
    unsigned int type9Size;
    UCHAR *t9;
    hDecoder = aacDecoder_Open (TRANSPORT_TYPE::TT_DRM, 3);

    // provide a default value for iAudioSampleRate in case we can't do better. TODO xHEAAC
    int iDefaultSampleRate;
    switch (AudioParam.eAudioSamplRate)
    {
    case CAudioParam::AS_12KHZ:
        iDefaultSampleRate = 12000;
        break;

    case CAudioParam::AS_24KHZ:
        iDefaultSampleRate = 24000;
        break;
    default:
        iDefaultSampleRate = 12000;
        break;
    }
    if (AudioParam.eSBRFlag == CAudioParam::SBR_USED)
    {
        iDefaultSampleRate = iDefaultSampleRate * 2;
    }

    if(hDecoder == nullptr) {
        iAudioSampleRate = iDefaultSampleRate;
        return false;
    }

    vector<uint8_t> type9 = AudioParam.getType9Bytes();
    type9Size = type9.size();
    t9 = &type9[0];

    // KiwiSDR NB: *this* is where CAudioSourceDecoder::inputSampleRate is set
    // as used in CAudioSourceDecoder::InitInternal()
    // Not obvious, but it is passed by pointer.
    
    //cerr << "type9 " << hex; for(size_t i=0; i<type9Size; i++) cerr << int(type9[i]) << " "; cerr << dec << endl;
    AAC_DECODER_ERROR err = aacDecoder_ConfigRaw(hDecoder, &t9, &type9Size);

    if (err == AAC_DEC_OK) {
        CStreamInfo *pinfo = aacDecoder_GetStreamInfo(hDecoder);
        if (pinfo==nullptr) {
            flog::warn("FDK DecOpen No stream info");
            iAudioSampleRate = iDefaultSampleRate;
            return true;// TODO
        }
    
        {
            std::ostringstream oss;
            std::streambuf* old_buf = std::cerr.rdbuf(oss.rdbuf());
            logAOT(oss, *pinfo);
            logFlags(oss, *pinfo);
            logNumbers(oss, *pinfo);
            std::cerr.rdbuf(old_buf);
            flog::info("FDK stream info: {}", oss.str());
        }
        iAudioSampleRate = pinfo->extSamplingRate;

        // KiwiSDR: fix to detect mono non-SBR, e.g. All India Radio, 7550 kHz
        if (iAudioSampleRate == 0) iAudioSampleRate = pinfo->aacSampleRate;
        if (iAudioSampleRate == 0) {
            flog::warn("FDK DecOpen codec returned sample rate = 0, using default {}", iDefaultSampleRate);
            iAudioSampleRate = iDefaultSampleRate; // get from AudioParam if codec couldn't get it
        }

        if(pinfo->aot == AUDIO_OBJECT_TYPE::AOT_USAC) bUsac = true;
#ifdef HAVE_USAC
        else if(pinfo->aot == AUDIO_OBJECT_TYPE::AOT_DRM_USAC) bUsac = true;
#endif
        else bUsac = false;

        return true;
    }

    iAudioSampleRate = iDefaultSampleRate;  // get from AudioParam if codec couldn't get it
    return true; // TODO
 }

CAudioCodec::EDecError FdkAacCodec::Decode(const vector<uint8_t>& audio_frame, uint8_t aac_crc_bits,
    CVector<_REAL>& left, CVector<_REAL>& right)
{
    writeFile(audio_frame);
    vector<uint8_t> data;
    uint8_t* pData;
    UINT bufferSize;

    if (bUsac) {
        pData = const_cast<uint8_t*>(&audio_frame[0]);
        bufferSize = audio_frame.size();
    }
    else {
        data.resize(audio_frame.size()+1);
        data[0] = aac_crc_bits;

        for (size_t i = 0; i < audio_frame.size(); i++)
            data[i + 1] = audio_frame[i];

        pData = &data[0];
        bufferSize = data.size();
    }
    UINT bytesValid = bufferSize;

    AAC_DECODER_ERROR err = aacDecoder_Fill(hDecoder, &pData, &bufferSize, &bytesValid);

    if (err != AAC_DEC_OK) {
        flog::error("FDK Fill failed: {}", int(err));
        return CAudioCodec::DECODER_ERROR_UNKNOWN;
    }

    CStreamInfo *pinfo = aacDecoder_GetStreamInfo(hDecoder);
    if (pinfo == nullptr) {
        cerr << "FDK No stream info" << endl;
        //return nullptr; this breaks everything!
    }

    if (pinfo->aacNumChannels == 0) {
        flog::warn("FDK zero output coded channels");
    }
    if (pinfo->numChannels == 0) {
        flog::warn("FDK zero output channels: err={}", int(err));
    }
    
    //cerr << "aac decode after fill bufferSize " << bufferSize << ", bytesValid " << bytesValid << endl;
    if (bytesValid != 0) {
        flog::error("FDK Unable to feed all {} input bytes, bytes left {}", bufferSize, bytesValid);
        return CAudioCodec::DECODER_ERROR_UNKNOWN;
    }

    size_t output_size = pinfo->frameSize * pinfo->numChannels;
    if (decode_buf.size() < output_size) {
        decode_buf.resize(output_size);
    } else {
        output_size = decode_buf.size();
    }

    err = aacDecoder_DecodeFrame(hDecoder, decode_buf.data(), output_size, 0);

    left.Init(pinfo->frameSize, 0.0);
    right.Init(pinfo->frameSize, 0.0);

    switch (err) {
    case AAC_DEC_OK:
        break;
    case AAC_DEC_CRC_ERROR:
        flog::error("FDK Decoder detected a CRC error in the bitstream.");
        return CAudioCodec::DECODER_ERROR_CRC;
    case AAC_DEC_PARSE_ERROR:
        flog::error("FDK Error parsing bitstream.");
        return CAudioCodec::DECODER_ERROR_CRC;
    default:
        flog::error("FDK Other error {}", int(err));
        return CAudioCodec::DECODER_ERROR_UNKNOWN;
    }

    if (pinfo->numChannels == 1)
    {
        for(int i = 0; i<pinfo->frameSize; i++) {
            left[int(i)] = _REAL(decode_buf[i]) / 2.0;
            right[int(i)] = _REAL(decode_buf[i]) / 2.0;
        }
    }
    else
    {
        for(int i = 0; i<pinfo->frameSize; i++) {
            left[int(i)] = _REAL(decode_buf[2*i]);
            right[int(i)] = _REAL(decode_buf[2*i+1]);
        }
    }

    return CAudioCodec::DECODER_ERROR_OK;
}

void
FdkAacCodec::DecClose()
{
    closeFile();
    if (hDecoder != nullptr)
	{
        aacDecoder_Close(hDecoder);
        hDecoder = nullptr;
	}
}

void
FdkAacCodec::DecUpdate(CAudioParam&)
{
}

string
FdkAacCodec::fileName(const CParameter& Parameters) const
{
    // Store AAC-data in file
    stringstream ss;
    ss << "aac_";

//    Parameters.Lock(); // TODO CAudioSourceDecoder::InitInternal() already have the lock
    if (Parameters.
            Service[Parameters.GetCurSelAudioService()].AudioParam.
            eAudioSamplRate == CAudioParam::AS_12KHZ)
    {
        ss << "12kHz_";
    }
    else
        ss << "24kHz_";

    switch (Parameters.
            Service[Parameters.GetCurSelAudioService()].
            AudioParam.eAudioMode)
    {
    case CAudioParam::AM_MONO:
        ss << "mono";
        break;

    case CAudioParam::AM_P_STEREO:
        ss << "pstereo";
        break;

    case CAudioParam::AM_STEREO:
        ss << "stereo";
        break;
    case CAudioParam::AM_RESERVED:;
    }

    if (Parameters.
            Service[Parameters.GetCurSelAudioService()].AudioParam.
            eSBRFlag == CAudioParam::SBR_USED)
    {
        ss << "_sbr";
    }
    ss << ".dat";

    return ss.str();
}

