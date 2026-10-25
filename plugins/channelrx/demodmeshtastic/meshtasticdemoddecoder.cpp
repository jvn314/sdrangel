///////////////////////////////////////////////////////////////////////////////////
// Copyright (C) 2020 Edouard Griffiths, F4EXB <f4exb06@gmail.com>               //
//                                                                               //
// This program is free software; you can redistribute it and/or modify          //
// it under the terms of the GNU General Public License as published by          //
// the Free Software Foundation as version 3 of the License, or                  //
// (at your option) any later version.                                           //
//                                                                               //
// This program is distributed in the hope that it will be useful,               //
// but WITHOUT ANY WARRANTY; without even the implied warranty of                //
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the                  //
// GNU General Public License V3 for more details.                               //
//                                                                               //
// You should have received a copy of the GNU General Public License             //
// along with this program. If not, see <http://www.gnu.org/licenses/>.          //
///////////////////////////////////////////////////////////////////////////////////

#include <QTime>
#include <algorithm>
#include <cmath>

#include "meshtasticdemoddecoder.h"
#include "meshtasticdemoddecoderlora.h"
#include "meshtasticdemodmsg.h"

MeshtasticDemodDecoder::MeshtasticDemodDecoder() :
    m_codingScheme(MeshtasticDemodSettings::CodingLoRa),
    m_spreadFactor(0U),
    m_deBits(0U),
    m_nbSymbolBits(5),
    m_nbParityBits(1),
    m_hasCRC(true),
    m_hasHeader(true),
    m_packetLength(0U),
    m_loRaBandwidth(250000U),
    m_nbSymbols(0U),
    m_nbCodewords(0U),
    m_earlyEOM(false),
    m_headerParityStatus((int) MeshtasticDemodSettings::ParityUndefined),
    m_headerCRCStatus(false),
    m_payloadParityStatus((int) MeshtasticDemodSettings::ParityUndefined),
    m_payloadCRCStatus(false),
    m_pipelineId(-1),
    m_outputMessageQueue(nullptr),
    m_headerFeedbackMessageQueue(nullptr)
{
    connect(&m_inputMessageQueue, SIGNAL(messageEnqueued()), this, SLOT(handleInputMessages()));
}

MeshtasticDemodDecoder::~MeshtasticDemodDecoder()
{}

void MeshtasticDemodDecoder::setNbSymbolBits(unsigned int spreadFactor, unsigned int deBits)
{
    m_spreadFactor = spreadFactor;

    if (deBits >= spreadFactor) {
        m_deBits = m_spreadFactor - 1;
    } else {
        m_deBits = deBits;
    }

    m_nbSymbolBits = m_spreadFactor - m_deBits;
}

void MeshtasticDemodDecoder::decodeSymbols(const std::vector<unsigned short>& symbols, QByteArray& bytes)
{
    if (m_nbSymbolBits >= 5)
    {
        unsigned int headerNbSymbolBits;

        if (m_hasHeader && (m_spreadFactor > 2U)) {
            headerNbSymbolBits = m_spreadFactor - 2U;
        } else {
            headerNbSymbolBits = m_nbSymbolBits;
        }

        MeshtasticDemodDecoderLoRa::decodeBytes(
            bytes,
            symbols,
            m_nbSymbolBits,
            headerNbSymbolBits,
            m_hasHeader,
            m_hasCRC,
            m_nbParityBits,
            m_packetLength,
            m_earlyEOM,
            m_headerParityStatus,
            m_headerCRCStatus,
            m_payloadParityStatus,
            m_payloadCRCStatus
        );

        MeshtasticDemodDecoderLoRa::getCodingMetrics(
            m_nbSymbolBits,
            headerNbSymbolBits,
            m_nbParityBits,
            m_packetLength,
            m_hasHeader,
            m_hasCRC,
            m_nbSymbols,
            m_nbCodewords
        );
    }
}

bool MeshtasticDemodDecoder::handleMessage(const Message& cmd)
{
    if (MeshtasticDemodMsg::MsgLoRaHeaderProbe::match(cmd))
    {
        MeshtasticDemodMsg::MsgLoRaHeaderProbe& msg = (MeshtasticDemodMsg::MsgLoRaHeaderProbe&) cmd;
        const std::vector<unsigned short>& symbols = msg.getSymbols();

        bool hasCRC = msg.getHasCRC();
        unsigned int nbParityBits = m_nbParityBits;
        unsigned int packetLength = m_packetLength;
        int headerParityStatus = (int) MeshtasticDemodSettings::ParityUndefined;
        bool headerCRCStatus = false;
        bool ldro = false;
        unsigned int expectedSymbols = 0U;
        bool valid = false;

        if (symbols.size() >= 8U && msg.getHasHeader())
        {
            MeshtasticDemodDecoderLoRa::decodeHeader(
                symbols,
                msg.getHeaderNbSymbolBits(),
                hasCRC,
                nbParityBits,
                packetLength,
                headerParityStatus,
                headerCRCStatus
            );

            if (headerCRCStatus && (packetLength > 0U) && (nbParityBits >= 1U) && (nbParityBits <= 4U))
            {
                const unsigned int spreadFactor = msg.getSpreadFactor();
                const unsigned int bandwidth = msg.getBandwidth() > 0U ? msg.getBandwidth() : m_loRaBandwidth;
                ldro = ((1U << spreadFactor) * 1000.0 / static_cast<double>(std::max(1U, bandwidth))) > 16.0;
                const int denom = static_cast<int>(spreadFactor) - (ldro ? 2 : 0);

                if (denom > 0)
                {
                    const int numerator =
                        2 * static_cast<int>(packetLength)
                        - static_cast<int>(spreadFactor)
                        + 2
                        + 5 // explicit header path (!impl_head)
                        + (hasCRC ? 4 : 0);
                    const int payloadBlocks = std::max(0, static_cast<int>(std::ceil(static_cast<double>(numerator) / static_cast<double>(denom))));
                    expectedSymbols = 8U + static_cast<unsigned int>(payloadBlocks) * (4U + nbParityBits);
                    valid = expectedSymbols >= 8U;
                }
            }
        }

        if (m_headerFeedbackMessageQueue)
        {
            MeshtasticDemodMsg::MsgLoRaHeaderFeedback *feedback = MeshtasticDemodMsg::MsgLoRaHeaderFeedback::create(
                msg.getFrameId(),
                valid,
                hasCRC,
                nbParityBits,
                packetLength,
                ldro,
                expectedSymbols,
                headerParityStatus,
                headerCRCStatus
            );
            m_headerFeedbackMessageQueue->push(feedback);
            qDebug("MeshtasticDemodDecoder::handleMessage: header probe frameId=%u valid=%d len=%u CR=%u expected=%u",
                msg.getFrameId(), valid ? 1 : 0, packetLength, nbParityBits, expectedSymbols);
        }

        return true;
    }
    else if (MeshtasticDemodMsg::MsgDecodeSymbols::match(cmd))
    {
        qDebug("MeshtasticDemodDecoder::handleMessage: MsgDecodeSymbols");
        MeshtasticDemodMsg::MsgDecodeSymbols& msg = (MeshtasticDemodMsg::MsgDecodeSymbols&) cmd;
        float msgSignalDb = msg.getSingalDb();
        float msgNoiseDb = msg.getNoiseDb();
        unsigned int msgSyncWord = msg.getSyncWord();
        QDateTime dt = QDateTime::currentDateTime();
        QString msgTimestamp = dt.toString(Qt::ISODateWithMs);

        QByteArray msgBytes;
        const std::vector<std::vector<float>>& msgMags = msg.getMagnitudes();
        const bool canSoftDecode = !msgMags.empty()
            && (msgMags.size() >= msg.getSymbols().size())
            && (m_spreadFactor >= 5U)
            && (m_loRaBandwidth > 0U);

        // Track which existing decode/recovery path supplies the final result.
        const char *decodePath = canSoftDecode ? "soft" : "hard";
        bool hardFallbackTried = false;
        unsigned int recoveryAttempts = 0U;
        int recoveryDelta = 0;

        struct LoRaDecodeState
        {
            QByteArray bytes;
            bool hasCRC;
            unsigned int nbParityBits;
            unsigned int packetLength;
            unsigned int nbSymbols;
            unsigned int nbCodewords;
            bool earlyEOM;
            int headerParityStatus;
            bool headerCRCStatus;
            int payloadParityStatus;
            bool payloadCRCStatus;
        };

        auto captureLoRaState = [this](const QByteArray& bytes) -> LoRaDecodeState {
            LoRaDecodeState s;
            s.bytes = bytes;
            s.hasCRC = m_hasCRC;
            s.nbParityBits = m_nbParityBits;
            s.packetLength = m_packetLength;
            s.nbSymbols = m_nbSymbols;
            s.nbCodewords = m_nbCodewords;
            s.earlyEOM = m_earlyEOM;
            s.headerParityStatus = m_headerParityStatus;
            s.headerCRCStatus = m_headerCRCStatus;
            s.payloadParityStatus = m_payloadParityStatus;
            s.payloadCRCStatus = m_payloadCRCStatus;
            return s;
        };

        auto restoreLoRaState = [this, &msgBytes](const LoRaDecodeState& s) {
            msgBytes = s.bytes;
            m_hasCRC = s.hasCRC;
            m_nbParityBits = s.nbParityBits;
            m_packetLength = s.packetLength;
            m_nbSymbols = s.nbSymbols;
            m_nbCodewords = s.nbCodewords;
            m_earlyEOM = s.earlyEOM;
            m_headerParityStatus = s.headerParityStatus;
            m_headerCRCStatus = s.headerCRCStatus;
            m_payloadParityStatus = s.payloadParityStatus;
            m_payloadCRCStatus = s.payloadCRCStatus;
        };

        // Record each existing attempt before state restoration can hide its outcome.
        auto logLoRaAttempt = [&msg](const char *path, int delta, const LoRaDecodeState& state) {
            qDebug("MeshtasticDemodDecoder::decodeAttempt: frameId=%u path=%s delta=%d symbols=%zu bytes=%lld earlyEOM=%d hCRC=%d pCRC=%d hasCRC=%d length=%u cr=%u decoderExpectedSymbols=%u",
                msg.getFrameId(), path, delta, msg.getSymbols().size(),
                static_cast<long long>(state.bytes.size()), state.earlyEOM ? 1 : 0,
                state.headerCRCStatus ? 1 : 0, state.payloadCRCStatus ? 1 : 0,
                state.hasCRC ? 1 : 0, state.packetLength, state.nbParityBits, state.nbSymbols);
        };

        if (canSoftDecode)
        {
            unsigned int headerNbSymbolBits;

            if (m_hasHeader && (m_spreadFactor > 2U)) {
                headerNbSymbolBits = m_spreadFactor - 2U;
            } else {
                headerNbSymbolBits = m_nbSymbolBits;
            }

            MeshtasticDemodDecoderLoRa::decodeBytesSoft(
                msgBytes,
                msgMags,
                msg.getSymbols(),
                m_spreadFactor,
                m_loRaBandwidth,
                m_nbSymbolBits,
                headerNbSymbolBits,
                m_hasHeader,
                m_hasCRC,
                m_nbParityBits,
                m_packetLength,
                m_earlyEOM,
                m_headerParityStatus,
                m_headerCRCStatus,
                m_payloadParityStatus,
                m_payloadCRCStatus
            );

            MeshtasticDemodDecoderLoRa::getCodingMetrics(
                m_nbSymbolBits,
                headerNbSymbolBits,
                m_nbParityBits,
                m_packetLength,
                m_hasHeader,
                m_hasCRC,
                m_nbSymbols,
                m_nbCodewords
            );

            const LoRaDecodeState softState = captureLoRaState(msgBytes);
            logLoRaAttempt("soft", 0, softState);

            // Soft path is canonical for gr-lora_sdr, but if this approximation misses CRC
            // on noisy captures, retry hard decode once and keep whichever path validates.
            if (m_hasCRC && !m_payloadCRCStatus)
            {
                hardFallbackTried = true;
                QByteArray hardBytes;
                decodeSymbols(msg.getSymbols(), hardBytes); // hard path updates decoder state
                const LoRaDecodeState hardState = captureLoRaState(hardBytes);
                logLoRaAttempt("hardUnshifted", 0, hardState);

                if (hardState.payloadCRCStatus) {
                    decodePath = "hardFallback";
                    restoreLoRaState(hardState);
                } else {
                    restoreLoRaState(softState);
                }
            }
        }
        else
        {
            decodeSymbols(msg.getSymbols(), msgBytes);
            logLoRaAttempt("hardUnshifted", 0, captureLoRaState(msgBytes));
        }

        // Preserve the unshifted result for comparison with recovery and the sink's header lock.
        const unsigned int unshiftedLength = m_packetLength;
        const bool unshiftedHeaderCRC = m_headerCRCStatus;
        const bool unshiftedPayloadCRC = m_payloadCRCStatus;

        if (m_hasCRC && !m_payloadCRCStatus && (m_spreadFactor >= 5U))
        {
            const LoRaDecodeState baseState = captureLoRaState(msgBytes);
            const unsigned int headerNbSymbolBits = (m_hasHeader && (m_spreadFactor > 2U))
                ? (m_spreadFactor - 2U)
                : m_nbSymbolBits;
            bool recovered = false;

            for (int delta : {-1, 1})
            {
                ++recoveryAttempts;
                std::vector<unsigned short> shifted = msg.getSymbols();

                for (size_t i = 0; i < shifted.size(); i++)
                {
                    const bool isHeader = m_hasHeader && (i < 8U);
                    const unsigned int bits = isHeader ? headerNbSymbolBits : m_nbSymbolBits;
                    const unsigned int mod = 1U << std::max(1U, bits);
                    const int s = static_cast<int>(shifted[i]);
                    const int v = (s + delta) % static_cast<int>(mod);
                    shifted[i] = static_cast<unsigned short>(v < 0 ? (v + static_cast<int>(mod)) : v);
                }

                QByteArray shiftedBytes;
                decodeSymbols(shifted, shiftedBytes); // hard-path decode with adjusted symbol indices
                const LoRaDecodeState shiftedState = captureLoRaState(shiftedBytes);
                logLoRaAttempt("shiftedHard", delta, shiftedState);

                if (shiftedState.payloadCRCStatus)
                {
                    decodePath = "shiftedHard";
                    recoveryDelta = delta;
                    restoreLoRaState(shiftedState);
                    recovered = true;
                    break;
                }
            }

            if (!recovered) {
                restoreLoRaState(baseState);
            }
        }

        // Use the same signal/noise estimates carried to packet reports, including failed frames.
        const float msgSnrDb = msgSignalDb - msgNoiseDb;
        qDebug(
            "MeshtasticDemodDecoder::handleMessage: decode frameId=%u symbols=%zu bytes=%lld earlyEOM=%d hCRC=%d pCRC=%d hParity=%d pParity=%d signalDb=%.2f noiseDb=%.2f snrDb=%.2f path=%s hardFallbackTried=%d recoveryAttempts=%u recoveryDelta=%d unshiftedLength=%u unshiftedHCRC=%d unshiftedPCRC=%d length=%u hasCRC=%d cr=%u decoderExpectedSymbols=%u",
            msg.getFrameId(),
            msg.getSymbols().size(),
            static_cast<long long>(msgBytes.size()),
            m_earlyEOM ? 1 : 0,
            m_headerCRCStatus ? 1 : 0,
            m_payloadCRCStatus ? 1 : 0,
            m_headerParityStatus,
            m_payloadParityStatus,
            msgSignalDb, msgNoiseDb, msgSnrDb, decodePath,
            hardFallbackTried ? 1 : 0, recoveryAttempts, recoveryDelta,
            unshiftedLength, unshiftedHeaderCRC ? 1 : 0, unshiftedPayloadCRC ? 1 : 0,
            m_packetLength, m_hasCRC ? 1 : 0, m_nbParityBits, m_nbSymbols
        );

        if (m_outputMessageQueue)
        {
            qDebug(
                "MeshtasticDemodDecoder::handleMessage: push report name=%s ts=%s bytes=%lld pCRC=%d",
                qPrintable(m_pipelineName),
                qPrintable(msgTimestamp),
                static_cast<long long>(msgBytes.size()),
                m_payloadCRCStatus ? 1 : 0
            );
            MeshtasticDemodMsg::MsgReportDecodeBytes *outputMsg = MeshtasticDemodMsg::MsgReportDecodeBytes::create(msgBytes);
            outputMsg->setFrameId(msg.getFrameId());
            outputMsg->setSyncWord(msgSyncWord);
            outputMsg->setSignalDb(msgSignalDb);
            outputMsg->setNoiseDb(msgNoiseDb);
            // Forward this frame's carrier frequency offset to the decode report.
            outputMsg->setCfoHz(msg.getCfoHz());
            outputMsg->setMsgTimestamp(msgTimestamp);
            outputMsg->setPacketSize(getPacketLength());
            outputMsg->setNbParityBits(getNbParityBits());
            outputMsg->setHasCRC(getHasCRC());
            outputMsg->setNbSymbols(getNbSymbols());
            outputMsg->setNbCodewords(getNbCodewords());
            outputMsg->setEarlyEOM(getEarlyEOM());
            outputMsg->setHeaderParityStatus(getHeaderParityStatus());
            outputMsg->setHeaderCRCStatus(getHeaderCRCStatus());
            outputMsg->setPayloadParityStatus(getPayloadParityStatus());
            outputMsg->setPayloadCRCStatus(getPayloadCRCStatus());
            outputMsg->setPipelineMetadata(m_pipelineId, m_pipelineName, m_pipelinePreset);
            outputMsg->setDechirpedSpectrum(msg.getDechirpedSpectrum());
            m_outputMessageQueue->push(outputMsg);
        }

        return true;
    }

    return false;
}

void MeshtasticDemodDecoder::handleInputMessages()
{
	Message* message;

	while ((message = m_inputMessageQueue.pop()))
	{
		if (!handleMessage(*message)) {
			qDebug("%s: unhandled message: %s", Q_FUNC_INFO, message->getIdentifier());
		}
		delete message;
	}
}
