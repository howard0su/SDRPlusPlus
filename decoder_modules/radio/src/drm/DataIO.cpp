/******************************************************************************\
 * Technische Universitaet Darmstadt, Institut fuer Nachrichtentechnik
 * Copyright (c) 2001-2006
 *
 * Author(s):
 *	Volker Fischer
 *
 * Description:
 *
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

#include "DataIO.h"
//#include "timer.h"
#include <stdio.h>

#include <iomanip>
#include <time.h>
#include "matlib/MatlibSigProToolbox.h"

using namespace std;

/* Implementation *************************************************************/
/******************************************************************************\
* MSC data																	   *
\******************************************************************************/

/******************************************************************************\
* FAC data																	   *
\******************************************************************************/
/* Transmitter */
void CGenerateFACData::ProcessDataInternal(CParameter& TransmParam)
{
    FACTransmit.FACParam(pvecOutputData, TransmParam);
}

void CGenerateFACData::InitInternal(CParameter& TransmParam)
{
    FACTransmit.Init(TransmParam);

    /* Define block-size for output */
    iOutputBlockSize = NUM_FAC_BITS_PER_BLOCK;
}

/* Receiver */
void CUtilizeFACData::ProcessDataInternal(CParameter& Parameters)
{
    /* Do not use received FAC data in case of simulation */
    if (bSyncInput == false)
    {
        bCRCOk = FACReceive.FACParam(pvecInputData, Parameters);
        /* Set FAC status for RSCI, log file & GUI */
        if (bCRCOk)
            Parameters.ReceiveStatus.FAC.SetStatus(RX_OK);
        else
            Parameters.ReceiveStatus.FAC.SetStatus(CRC_ERROR);
    }

    if ((bSyncInput) || (bCRCOk == false))
    {
        /* If FAC CRC check failed we should increase the frame-counter
           manually. If only FAC data was corrupted, the others can still
           decode if they have the right frame number. In case of simulation
           no FAC data is used, we have to increase the counter here */
        Parameters.iFrameIDReceiv++;

        if (Parameters.iFrameIDReceiv == NUM_FRAMES_IN_SUPERFRAME)
            Parameters.iFrameIDReceiv = 0;
    }
}

void CUtilizeFACData::InitInternal(CParameter& Parameters)
{

    // This should be in FAC class in an Init() routine which has to be defined, this
    // would be cleaner code! TODO
    /* Init frame ID so that a "0" comes after increasing the init value once */
    Parameters.iFrameIDReceiv = NUM_FRAMES_IN_SUPERFRAME - 1;

    /* Reset flag */
    bCRCOk = false;

    /* Define block-size for input */
    iInputBlockSize = NUM_FAC_BITS_PER_BLOCK;
}


/******************************************************************************\
* SDC data																	   *
\******************************************************************************/
/* Transmitter */
void CGenerateSDCData::ProcessDataInternal(CParameter& TransmParam)
{
    SDCTransmit.SDCParam(pvecOutputData, TransmParam);
}

void CGenerateSDCData::InitInternal(CParameter& TransmParam)
{
    /* Define block-size for output */
    iOutputBlockSize = TransmParam.iNumSDCBitsPerSFrame;
}

/* Receiver */
void CUtilizeSDCData::ProcessDataInternal(CParameter& Parameters)
{
    //    bool bSDCOK = false;

    /* Decode SDC block and return CRC status */
    CSDCReceive::ERetStatus eStatus = SDCReceive.SDCParam(pvecInputData, Parameters);

    Parameters.Lock();
    switch (eStatus)
    {
    case CSDCReceive::SR_OK:
        Parameters.ReceiveStatus.SDC.SetStatus(RX_OK);
        //        bSDCOK = true;
        break;

    case CSDCReceive::SR_BAD_CRC:
        /* SDC block depends on only a few parameters: robustness mode,
           DRM bandwidth and coding scheme (can be 4 or 16 QAM). If we
           initialize these parameters with resonable parameters it might
           be possible that these are the correct parameters. Therefore
           try to decode SDC even in case FAC wasn't decoded. That might
           speed up the DRM signal acqisition. But quite often it is the
           case that the parameters are not correct. In this case do not
           show a red light if SDC CRC was not ok */
        if (bFirstBlock == false)
            Parameters.ReceiveStatus.SDC.SetStatus(CRC_ERROR);
        break;

    case CSDCReceive::SR_BAD_DATA:
        /* CRC was ok but data seems to be incorrect */
        Parameters.ReceiveStatus.SDC.SetStatus(DATA_ERROR);
        break;
    }
    Parameters.Unlock();

    /* Reset "first block" flag */
    bFirstBlock = false;
}

void CUtilizeSDCData::InitInternal(CParameter& Parameters)
{
    /* Init "first block" flag */
    bFirstBlock = true;

    /* Define block-size for input */
    iInputBlockSize = Parameters.iNumSDCBitsPerSFrame;
}
