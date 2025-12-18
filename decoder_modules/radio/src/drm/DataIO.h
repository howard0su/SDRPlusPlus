/******************************************************************************\
 * Technische Universitaet Darmstadt, Institut fuer Nachrichtentechnik
 * Copyright (c) 2001-2005
 *
 * Author(s):
 *	Volker Fischer, Andrew Murphy
 *
 * Description:
 *	See Data.cpp
 *
 * 11/21/2005 Andrew Murphy, BBC Research & Development, 2005
 *	- Addition GetSDCReceive(), Added CSplit class
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

#if !defined(DATA_H__3B0BA660_CA63_4344_BB2B_23E7A0D31912__INCLUDED_)
#define DATA_H__3B0BA660_CA63_4344_BB2B_23E7A0D31912__INCLUDED_

#include "Parameter.h"
#include "util/Modul.h"
#include "FAC/FAC.h"
#include "SDC/SDC.h"
#include "TextMessage.h"
#include "util/AudioFile.h"
#include "util/Utilities.h"

/* Definitions ****************************************************************/

/* In case of random-noise, define number of blocks */
#define DEFAULT_NUM_SIM_BLOCKS		50

/* Time span used for averaging the audio spectrum. Shall be higher than the
   400 ms DRM audio block */
#define TIME_AV_AUDIO_SPECT_MS		500 /* ms */

/* Normalization constant for two mixed signals. If this constant is 2, no
   overrun of the "short" variable can happen but signal has quite much lower
   power -> compromise */
#define MIX_OUT_CHAN_NORM_CONST		((_REAL) 1.0 / sqrt((_REAL) 2.0))


/* Classes ********************************************************************/
/* MSC ---------------------------------------------------------------------- */

/* FAC ---------------------------------------------------------------------- */
class CGenerateFACData : public CTransmitterModul<_BINARY, _BINARY>
{
public:
    CGenerateFACData() {}
    virtual ~CGenerateFACData() {}

protected:
    CFACTransmit FACTransmit;

    virtual void InitInternal(CParameter& TransmParam);
    virtual void ProcessDataInternal(CParameter& TransmParam);
};

class CUtilizeFACData : public CReceiverModul<_BINARY, _BINARY>
{
public:
    CUtilizeFACData() :
            bSyncInput(false), bCRCOk(false) {}
    virtual ~CUtilizeFACData() {}

    /* To set the module up for synchronized DRM input data stream */
    void SetSyncInput(bool bNewS) {
        bSyncInput = bNewS;
    }

    bool GetCRCOk() const {
        return bCRCOk;
    }

protected:
    CFACReceive FACReceive;
    bool	bSyncInput;
    bool	bCRCOk;

    virtual void InitInternal(CParameter& Parameters);
    virtual void ProcessDataInternal(CParameter& Parameters);
};


/* SDC ---------------------------------------------------------------------- */
class CUtilizeSDCData : public CReceiverModul<_BINARY, _BINARY>
{
public:
    CUtilizeSDCData() {}
    virtual ~CUtilizeSDCData() {}

    CSDCReceive* GetSDCReceive() {
        return &SDCReceive;
    }

protected:
    CSDCReceive SDCReceive;
    bool	bFirstBlock;

    virtual void InitInternal(CParameter& Parameters);
    virtual void ProcessDataInternal(CParameter& Parameters);
};


/******************************************************************************\
* Data type conversion classes needed for simulation and AMSS decoding         *
\******************************************************************************/
/* Conversion from channel output to resample module input */
class CDataConvChanResam : public CReceiverModul<CChanSimDataMod, _REAL>
{
protected:
    virtual void InitInternal(CParameter& Parameters)
    {
        iInputBlockSize = Parameters.CellMappingTable.iSymbolBlockSize;
        iOutputBlockSize = Parameters.CellMappingTable.iSymbolBlockSize;
    }
    virtual void ProcessDataInternal(CParameter&)
    {
        for (int i = 0; i < iOutputBlockSize; i++)
            (*pvecOutputData)[i] = (*pvecInputData)[i].tOut;
    }
};

/* Takes an input buffer and splits it 2 ways */
class CSplit: public CReceiverModul<_REAL, _REAL>
{
protected:
    virtual void InitInternal(CParameter& Parameters)
    {
        iInputBlockSize = Parameters.CellMappingTable.iSymbolBlockSize;
        iOutputBlockSize = Parameters.CellMappingTable.iSymbolBlockSize;
        iOutputBlockSize2 = Parameters.CellMappingTable.iSymbolBlockSize;
    }
    virtual void ProcessDataInternal(CParameter&)
    {
        for (int i = 0; i < iInputBlockSize; i++)
        {
            (*pvecOutputData)[i] = (*pvecInputData)[i];
            (*pvecOutputData2)[i] = (*pvecInputData)[i];
        }
    }
};


#endif // !defined(DATA_H__3B0BA660_CA63_4344_BB2B_23E7A0D31912__INCLUDED_)
