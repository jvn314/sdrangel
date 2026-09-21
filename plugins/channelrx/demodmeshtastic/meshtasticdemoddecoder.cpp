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

        const LoRaDecodeState entryState = captureLoRaState(QByteArray());
        QString productionCandidateId = QStringLiteral("hard");

        if (canSoftDecode)
        {
            productionCandidateId = QStringLiteral("soft");
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

            // Soft path is canonical for gr-lora_sdr, but if this approximation misses CRC
            // on noisy captures, retry hard decode once and keep whichever path validates.
            if (m_hasCRC && !m_payloadCRCStatus)
            {
                QByteArray hardBytes;
                decodeSymbols(msg.getSymbols(), hardBytes); // hard path updates decoder state
                const LoRaDecodeState hardState = captureLoRaState(hardBytes);

                if (hardState.payloadCRCStatus) {
                    restoreLoRaState(hardState);
                    productionCandidateId = QStringLiteral("hard");
                } else {
                    restoreLoRaState(softState);
                }
            }
        }
        else
        {
            decodeSymbols(msg.getSymbols(), msgBytes);
        }

        // Classify the base decode before recovery.
        // na = bin-recovery classification does not apply.
        QString binFix = QStringLiteral("na");

        if (m_hasCRC)
        {
            if (m_payloadCRCStatus)
            {
                // c0 = base decode passed without bin correction.
                binFix = QStringLiteral("c0");
            }
            else
            {
                // fn = decode failed before the recovery path was entered.
                binFix = QStringLiteral("fn");
            }
        }
        else if (m_hasHeader && !m_headerCRCStatus)
        {
            // Explicit header failed validation, so its decoded CRC-enable bit is untrusted.
            // If that bit decoded as zero, the current recovery gate prevents a retry.
            // fn = decode failed before the recovery path was entered.
            binFix = QStringLiteral("fn");
        }

        // Symbol recovery retries packets that fail the normal CRC path.
        // The 8-symbol header is quantized more coarsely than the payload.
        // Therefore +1 shifts both regions, while -1 preserves the header and shifts the payload.
        if (m_hasCRC && !m_payloadCRCStatus && (m_spreadFactor >= 5U))
        {
            // fr = recovery was attempted but neither correction has validated yet.
            binFix = QStringLiteral("fr");

            const LoRaDecodeState baseState = captureLoRaState(msgBytes);
            const unsigned int headerNbSymbolBits = (m_hasHeader && (m_spreadFactor > 2U))
                ? (m_spreadFactor - 2U)
                : m_nbSymbolBits;
            bool recovered = false;

            // Positive-offset recovery:
            // Apply a +1 FFT-bin correction across the 8-symbol header and payload.
            {
                // Restore the original decode state before the positive-offset attempt.
                restoreLoRaState(baseState);
                std::vector<unsigned short> shifted = msg.getSymbols();

                for (size_t i = 0; i < shifted.size(); i++)
                {
                    const bool isHeader = m_hasHeader && (i < 8U);
                    const unsigned int bits = isHeader ? headerNbSymbolBits : m_nbSymbolBits;
                    const unsigned int mod = 1U << std::max(1U, bits);
                    const int s = static_cast<int>(shifted[i]);
                    const int v = (s + 1) % static_cast<int>(mod);
                    shifted[i] = static_cast<unsigned short>(v);
                }

                QByteArray shiftedBytes;
                decodeSymbols(shifted, shiftedBytes); // hard-path decode with adjusted symbol indices
                const LoRaDecodeState shiftedState = captureLoRaState(shiftedBytes);

                if ((!m_hasHeader || shiftedState.headerCRCStatus)
                    && shiftedState.hasCRC
                    && shiftedState.payloadCRCStatus)
                {
                    restoreLoRaState(shiftedState);
                    // cp = +1 symbol correction produced a CRC-valid decode.
                    binFix = QStringLiteral("cp");
                    productionCandidateId = QStringLiteral("plus1_bin");
                    recovered = true;
                }
            }

            // Negative-offset recovery:
            // Preserve the 8-symbol header and apply an FFT-bin correction of -1 only to payload symbols.
            if (!recovered)
            {
                // Restore the original decode state before the negative-offset attempt.
                restoreLoRaState(baseState);
                std::vector<unsigned short> shifted = msg.getSymbols();

                for (size_t i = m_hasHeader ? 8U : 0U; i < shifted.size(); i++)
                {
                    const unsigned int mod = 1U << std::max(1U, m_nbSymbolBits);
                    const int s = static_cast<int>(shifted[i]);
                    const int v = (s - 1) % static_cast<int>(mod);
                    shifted[i] = static_cast<unsigned short>(v < 0 ? (v + static_cast<int>(mod)) : v);
                }

                QByteArray shiftedBytes;
                decodeSymbols(shifted, shiftedBytes); // hard-path decode with adjusted symbol indices
                const LoRaDecodeState shiftedState = captureLoRaState(shiftedBytes);

                if ((!m_hasHeader || shiftedState.headerCRCStatus)
                    && shiftedState.hasCRC
                    && shiftedState.payloadCRCStatus)
                {
                    restoreLoRaState(shiftedState);
                    // cn = -1 payload-symbol correction produced a CRC-valid decode.
                    binFix = QStringLiteral("cn");
                    productionCandidateId = QStringLiteral("minus1_bin");
                    recovered = true;
                }
            }

            // Restore the original decode state when neither CRC-validated correction succeeds.
            if (!recovered) {
                restoreLoRaState(baseState);
                productionCandidateId = QStringLiteral("NONE");
            }
        }

        const LoRaDecodeState productionState = captureLoRaState(msgBytes);
        const QString productionHeaderSource =
            ((productionCandidateId == QStringLiteral("plus1_bin"))
                || (productionCandidateId == QStringLiteral("minus1_bin")))
            ? QStringLiteral("REDECODED")
            : ((productionCandidateId == QStringLiteral("soft"))
                || (productionCandidateId == QStringLiteral("hard")))
                ? QStringLiteral("BASE_VALIDATED")
                : QStringLiteral("NONE");

        auto makeTempResult = [](const QString& sourceRef,
                                 const QString& candidateId,
                                 const QString& headerSource,
                                 const QString& resultBinFix,
                                 const LoRaDecodeState& state) {
            MeshtasticDemodMsg::TempDecodePathResult result;
            result.sourceRef = sourceRef;
            result.candidateId = candidateId;
            result.headerSource = headerSource;
            result.binFix = resultBinFix;
            result.bytes = state.bytes;
            result.hasCRC = state.hasCRC;
            result.nbParityBits = state.nbParityBits;
            result.packetLength = state.packetLength;
            result.nbSymbols = state.nbSymbols;
            result.nbCodewords = state.nbCodewords;
            result.earlyEOM = state.earlyEOM;
            result.headerParityStatus = state.headerParityStatus;
            result.headerCRCStatus = state.headerCRCStatus;
            result.payloadParityStatus = state.payloadParityStatus;
            result.payloadCRCStatus = state.payloadCRCStatus;
            return result;
        };

        auto runHistoricalPath = [&](bool fftBinFixPolicy,
                                     const QString& sourceRef) -> MeshtasticDemodMsg::TempDecodePathResult
        {
            restoreLoRaState(entryState);
            QByteArray pathBytes;
            QString candidateId = canSoftDecode ? QStringLiteral("soft") : QStringLiteral("hard");

            if (canSoftDecode)
            {
                const unsigned int headerNbSymbolBits =
                    (m_hasHeader && (m_spreadFactor > 2U))
                    ? (m_spreadFactor - 2U)
                    : m_nbSymbolBits;

                MeshtasticDemodDecoderLoRa::decodeBytesSoft(
                    pathBytes,
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

                const LoRaDecodeState softState = captureLoRaState(pathBytes);

                if (m_hasCRC && !m_payloadCRCStatus)
                {
                    QByteArray hardBytes;
                    decodeSymbols(msg.getSymbols(), hardBytes);
                    const LoRaDecodeState hardState = captureLoRaState(hardBytes);

                    if (hardState.payloadCRCStatus)
                    {
                        pathBytes = hardBytes;
                        restoreLoRaState(hardState);
                        candidateId = QStringLiteral("hard");
                    }
                    else
                    {
                        pathBytes = softState.bytes;
                        restoreLoRaState(softState);
                        candidateId = QStringLiteral("soft");
                    }
                }
            }
            else
            {
                decodeSymbols(msg.getSymbols(), pathBytes);
            }

            QString pathBinFix = QStringLiteral("na");

            if (m_hasCRC) {
                pathBinFix = m_payloadCRCStatus ? QStringLiteral("c0") : QStringLiteral("fn");
            } else if (m_hasHeader && !m_headerCRCStatus) {
                pathBinFix = QStringLiteral("fn");
            }

            if (m_hasCRC && !m_payloadCRCStatus && (m_spreadFactor >= 5U))
            {
                pathBinFix = QStringLiteral("fr");
                const LoRaDecodeState baseState = captureLoRaState(pathBytes);
                const unsigned int headerNbSymbolBits =
                    (m_hasHeader && (m_spreadFactor > 2U))
                    ? (m_spreadFactor - 2U)
                    : m_nbSymbolBits;
                bool recovered = false;

                if (!fftBinFixPolicy)
                {
                    // Exact recovery policy at merge-base 31b60d86:
                    // try whole-frame -1 first, then whole-frame +1, and accept
                    // any candidate whose payload CRC validates.
                    for (int delta : {-1, 1})
                    {
                        restoreLoRaState(baseState);
                        std::vector<unsigned short> shifted = msg.getSymbols();

                        for (size_t i = 0; i < shifted.size(); ++i)
                        {
                            const bool isHeader = m_hasHeader && (i < 8U);
                            const unsigned int bits = isHeader ? headerNbSymbolBits : m_nbSymbolBits;
                            const unsigned int mod = 1U << std::max(1U, bits);
                            const int s = static_cast<int>(shifted[i]);
                            const int v = (s + delta) % static_cast<int>(mod);
                            shifted[i] = static_cast<unsigned short>(
                                v < 0 ? (v + static_cast<int>(mod)) : v);
                        }

                        QByteArray shiftedBytes;
                        decodeSymbols(shifted, shiftedBytes);
                        const LoRaDecodeState shiftedState = captureLoRaState(shiftedBytes);

                        if (shiftedState.payloadCRCStatus)
                        {
                            pathBytes = shiftedBytes;
                            restoreLoRaState(shiftedState);
                            pathBinFix = delta < 0 ? QStringLiteral("cn") : QStringLiteral("cp");
                            candidateId = delta < 0
                                ? QStringLiteral("minus1_bin")
                                : QStringLiteral("plus1_bin");
                            recovered = true;
                            break;
                        }
                    }
                }
                else
                {
                    // Exact recovery policy at fft-bin-fix b201d32a:
                    // +1 across header+payload with hardened acceptance, then
                    // -1 on payload only while preserving the original header.
                    restoreLoRaState(baseState);
                    std::vector<unsigned short> shifted = msg.getSymbols();

                    for (size_t i = 0; i < shifted.size(); ++i)
                    {
                        const bool isHeader = m_hasHeader && (i < 8U);
                        const unsigned int bits = isHeader ? headerNbSymbolBits : m_nbSymbolBits;
                        const unsigned int mod = 1U << std::max(1U, bits);
                        shifted[i] = static_cast<unsigned short>(
                            (static_cast<int>(shifted[i]) + 1) % static_cast<int>(mod));
                    }

                    QByteArray shiftedBytes;
                    decodeSymbols(shifted, shiftedBytes);
                    LoRaDecodeState shiftedState = captureLoRaState(shiftedBytes);

                    if ((!m_hasHeader || shiftedState.headerCRCStatus)
                        && shiftedState.hasCRC
                        && shiftedState.payloadCRCStatus)
                    {
                        pathBytes = shiftedBytes;
                        restoreLoRaState(shiftedState);
                        pathBinFix = QStringLiteral("cp");
                        candidateId = QStringLiteral("plus1_bin");
                        recovered = true;
                    }

                    if (!recovered)
                    {
                        restoreLoRaState(baseState);
                        shifted = msg.getSymbols();

                        for (size_t i = m_hasHeader ? 8U : 0U; i < shifted.size(); ++i)
                        {
                            const unsigned int mod = 1U << std::max(1U, m_nbSymbolBits);
                            const int v = (static_cast<int>(shifted[i]) - 1) % static_cast<int>(mod);
                            shifted[i] = static_cast<unsigned short>(
                                v < 0 ? (v + static_cast<int>(mod)) : v);
                        }

                        shiftedBytes.clear();
                        decodeSymbols(shifted, shiftedBytes);
                        shiftedState = captureLoRaState(shiftedBytes);

                        if ((!m_hasHeader || shiftedState.headerCRCStatus)
                            && shiftedState.hasCRC
                            && shiftedState.payloadCRCStatus)
                        {
                            pathBytes = shiftedBytes;
                            restoreLoRaState(shiftedState);
                            pathBinFix = QStringLiteral("cn");
                            candidateId = QStringLiteral("minus1_bin");
                            recovered = true;
                        }
                    }
                }

                if (!recovered)
                {
                    pathBytes = baseState.bytes;
                    restoreLoRaState(baseState);
                    candidateId = QStringLiteral("NONE");
                }
            }

            const LoRaDecodeState finalState = captureLoRaState(pathBytes);
            const QString headerSource =
                ((candidateId == QStringLiteral("plus1_bin"))
                    || (candidateId == QStringLiteral("minus1_bin")))
                ? QStringLiteral("REDECODED")
                : ((candidateId == QStringLiteral("soft"))
                    || (candidateId == QStringLiteral("hard")))
                    ? QStringLiteral("BASE_VALIDATED")
                    : QStringLiteral("NONE");

            return makeTempResult(sourceRef, candidateId, headerSource, pathBinFix, finalState);
        };

        MeshtasticDemodMsg::TempDecodeComparison tempComparison;
        tempComparison.valid = true;
        tempComparison.inputSymbols = msg.getSymbols();
        tempComparison.inputMagnitudeRows = static_cast<unsigned int>(msgMags.size());
        tempComparison.preBinfix = runHistoricalPath(
            false,
            QStringLiteral("31b60d86bfcf3e1b0d32ad5a1e4c5024a2e23706"));
        tempComparison.fftBinfix = runHistoricalPath(
            true,
            QStringLiteral("b201d32ae9b6a9ffe6fc19a535f0e9e98facbed4"));
        tempComparison.current = makeTempResult(
            QStringLiteral("04df1b7c44dae418c8f69c620b9d2859e5114c26"),
            productionCandidateId,
            productionHeaderSource,
            binFix,
            productionState);

        // Shadow decodes must never change production behavior or state.
        restoreLoRaState(productionState);

        qDebug(
            "MeshtasticDemodDecoder::handleMessage: decode symbols=%zu bytes=%lld earlyEOM=%d hCRC=%d pCRC=%d hParity=%d pParity=%d",
            msg.getSymbols().size(),
            static_cast<long long>(msgBytes.size()),
            m_earlyEOM ? 1 : 0,
            m_headerCRCStatus ? 1 : 0,
            m_payloadCRCStatus ? 1 : 0,
            m_headerParityStatus,
            m_payloadParityStatus
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
            outputMsg->setBinFix(binFix);
            outputMsg->setFrameId(msg.getFrameId());
            outputMsg->setSyncWord(msgSyncWord);
            outputMsg->setSignalDb(msgSignalDb);
            outputMsg->setNoiseDb(msgNoiseDb);
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
            // Preserve the sink's frame-time RF snapshot unchanged through the
            // decoder/report boundary.
            outputMsg->setRFMetadata(
                msg.getCenterFrequencyHz(),
                msg.getBandwidthHz(),
                msg.getSpreadFactor()
            );
            // Preserve the temporary frame-capture RF diagnostics unchanged through
            // the decoder/report boundary.
            outputMsg->setRFDiagnostics(
                msg.getTempDeviceCenterFrequencyHz(),
                msg.getTempInputFrequencyOffsetHz(),
                msg.getTempChannelFrequencyOffsetHz()
            );
            // Pipeline ID and name identify the runtime, but the preset is frame-time
            // configuration provenance and therefore comes from the frame snapshot.
            outputMsg->setPipelineMetadata(m_pipelineId, m_pipelineName, msg.getPipelinePreset());
            outputMsg->setDechirpedSpectrum(msg.getDechirpedSpectrum());
            outputMsg->setTempDecodeComparison(tempComparison);
            m_outputMessageQueue->push(outputMsg);
        }

        return true;
    }

    return false;
}

void MeshtasticDemodDecoder::handleInputMessages()
{
	Message* message;

	while ((message = m_inputMessageQueue.pop()) != nullptr)
	{
		if (handleMessage(*message)) {
			delete message;
		}
	}
}
