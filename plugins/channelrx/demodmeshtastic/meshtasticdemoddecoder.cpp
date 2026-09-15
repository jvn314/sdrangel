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
        float msgCfoHz = msg.getCfoHz();
        float msgSfoPpm = msg.getSfoPpm();
        unsigned int msgSyncWord = msg.getSyncWord();
        QDateTime dt = QDateTime::currentDateTime();
        QString msgTimestamp = dt.toString(Qt::ISODateWithMs);

        QByteArray msgBytes;
        const std::vector<std::vector<float>>& msgMags = msg.getMagnitudes();
        const std::vector<float>& msgSfoCumBefore = msg.getSfoCumBefore();
        const std::vector<int>& msgTimingStepAfterSymbol = msg.getTimingStepAfterSymbol();

        float fftMarginMinDb = 0.0f;
        double fftMarginSumDb = 0.0;
        unsigned int fftMarginCount = 0U;
        unsigned int fftMarginLt1Db = 0U;
        unsigned int fftMarginLt3Db = 0U;
        std::vector<MeshtasticDemodMsg::FftPeakDiagnostic> fftPeakDiagnostics;
        fftPeakDiagnostics.reserve(msgMags.size());
        std::vector<float> fftTotalPowers;
        fftTotalPowers.reserve(msgMags.size());
        float fftBestFractionMin = 1.0f;
        int fftBestFractionWorstSymbol = -1;

        for (size_t symbolIndex = 0; symbolIndex < msgMags.size(); ++symbolIndex)
        {
            const std::vector<float>& mags = msgMags[symbolIndex];

            if (mags.size() < 2U) {
                continue;
            }

            float best = 0.0f;
            float secondBest = 0.0f;
            float totalPower = 0.0f;
            int bestBin = -1;
            int secondBin = -1;

            for (size_t bin = 0; bin < mags.size(); ++bin)
            {
                const float mag = mags[bin];
                totalPower += std::max(0.0f, mag);

                if (mag > best)
                {
                    secondBest = best;
                    secondBin = bestBin;
                    best = mag;
                    bestBin = static_cast<int>(bin);
                }
                else if (mag > secondBest)
                {
                    secondBest = mag;
                    secondBin = static_cast<int>(bin);
                }
            }

            if (best <= 0.0f) {
                continue;
            }

            int secondOffset = 0;
            if ((bestBin >= 0) && (secondBin >= 0))
            {
                const int nBins = static_cast<int>(mags.size());
                secondOffset = secondBin - bestBin;
                if (secondOffset > (nBins / 2)) {
                    secondOffset -= nBins;
                } else if (secondOffset < -(nBins / 2)) {
                    secondOffset += nBins;
                }
            }

            MeshtasticDemodMsg::FftPeakDiagnostic peakDiagnostic;
            peakDiagnostic.bestBin = bestBin;
            peakDiagnostic.secondBin = secondBin;
            peakDiagnostic.secondOffset = secondOffset;
            peakDiagnostic.bestPower = best;
            peakDiagnostic.secondPower = secondBest;
            peakDiagnostic.totalPower = totalPower;
            peakDiagnostic.bestFraction =
                totalPower > 0.0f ? best / totalPower : 0.0f;
            peakDiagnostic.sfoCumBefore =
                symbolIndex < msgSfoCumBefore.size() ? msgSfoCumBefore[symbolIndex] : 0.0f;
            peakDiagnostic.timingStepAfterSymbol =
                symbolIndex < msgTimingStepAfterSymbol.size() ? msgTimingStepAfterSymbol[symbolIndex] : 0;
            fftPeakDiagnostics.push_back(peakDiagnostic);
            fftTotalPowers.push_back(totalPower);

            if ((fftBestFractionWorstSymbol < 0)
                || (peakDiagnostic.bestFraction < fftBestFractionMin))
            {
                fftBestFractionMin = peakDiagnostic.bestFraction;
                fftBestFractionWorstSymbol = static_cast<int>(symbolIndex);
            }

            const float marginDb =
                10.0f * std::log10(best / std::max(secondBest, 1.0e-30f));

            if ((fftMarginCount == 0U) || (marginDb < fftMarginMinDb)) {
                fftMarginMinDb = marginDb;
            }

            fftMarginSumDb += marginDb;
            fftMarginCount++;

            if (marginDb < 1.0f) {
                fftMarginLt1Db++;
            }

            if (marginDb < 3.0f) {
                fftMarginLt3Db++;
            }
        }

        const float fftMarginAvgDb =
            fftMarginCount > 0U
                ? static_cast<float>(fftMarginSumDb / fftMarginCount)
                : 0.0f;

        auto medianPositive = [](const std::vector<float>& values) -> float
        {
            std::vector<float> positive;
            positive.reserve(values.size());

            for (float value : values)
            {
                if (value > 0.0f) {
                    positive.push_back(value);
                }
            }

            if (positive.empty()) {
                return 0.0f;
            }

            std::sort(positive.begin(), positive.end());
            const size_t middle = positive.size() / 2U;

            if ((positive.size() & 1U) != 0U) {
                return positive[middle];
            }

            return 0.5f * (positive[middle - 1U] + positive[middle]);
        };

        float fftTotalPowerPeakToMedianDb = 0.0f;
        int fftTotalPowerPeakSymbol = -1;

        if (!fftTotalPowers.empty())
        {
            const float median = medianPositive(fftTotalPowers);
            const auto peakIt = std::max_element(fftTotalPowers.begin(), fftTotalPowers.end());

            if (peakIt != fftTotalPowers.end())
            {
                fftTotalPowerPeakSymbol = static_cast<int>(
                    std::distance(fftTotalPowers.begin(), peakIt));

                if ((median > 0.0f) && (*peakIt > 0.0f)) {
                    fftTotalPowerPeakToMedianDb =
                        10.0f * std::log10(*peakIt / median);
                }
            }
        }

        float samplePeakToRmsMaxDb = 0.0f;
        int samplePeakToRmsWorstSymbol = -1;
        float sampleMeanPowerPeakToMedianDb = 0.0f;
        int sampleMeanPowerPeakSymbol = -1;
        std::vector<float> sampleMeanPowers;
        sampleMeanPowers.reserve(msg.getTransientDiagnostics().size());

        for (size_t i = 0; i < msg.getTransientDiagnostics().size(); ++i)
        {
            const MeshtasticDemodMsg::SymbolTransientDiagnostic& diagnostic =
                msg.getTransientDiagnostics()[i];
            sampleMeanPowers.push_back(diagnostic.meanPower);

            if ((diagnostic.peakPower > 0.0f) && (diagnostic.meanPower > 0.0f))
            {
                const float peakToRmsDb =
                    10.0f * std::log10(diagnostic.peakPower / diagnostic.meanPower);

                if ((samplePeakToRmsWorstSymbol < 0)
                    || (peakToRmsDb > samplePeakToRmsMaxDb))
                {
                    samplePeakToRmsMaxDb = peakToRmsDb;
                    samplePeakToRmsWorstSymbol = static_cast<int>(i);
                }
            }
        }

        if (!sampleMeanPowers.empty())
        {
            const float median = medianPositive(sampleMeanPowers);
            const auto peakIt = std::max_element(sampleMeanPowers.begin(), sampleMeanPowers.end());

            if (peakIt != sampleMeanPowers.end())
            {
                sampleMeanPowerPeakSymbol = static_cast<int>(
                    std::distance(sampleMeanPowers.begin(), peakIt));

                if ((median > 0.0f) && (*peakIt > 0.0f)) {
                    sampleMeanPowerPeakToMedianDb =
                        10.0f * std::log10(*peakIt / median);
                }
            }
        }

        if (fftBestFractionWorstSymbol < 0) {
            fftBestFractionMin = 0.0f;
        }

        const bool canSoftDecode = !msgMags.empty()
            && (msgMags.size() >= msg.getSymbols().size())
            && (m_spreadFactor >= 5U)
            && (m_loRaBandwidth > 0U);

        std::vector<int> headerRawResidues;
        std::vector<unsigned short> headerDecodedSymbols;
        unsigned int headerRawResidueModulus = 0U;
        bool headerRawResidueModulusConsistent = true;

        for (const MeshtasticDemodMsg::SymbolMappingDiagnostic& diagnostic : msg.getSymbolMappingDiagnostics())
        {
            if (!diagnostic.headerSymbol || (headerRawResidues.size() >= 8U)) {
                continue;
            }

            const unsigned int modulus = std::max(1U, diagnostic.spread);

            if (headerRawResidues.empty()) {
                headerRawResidueModulus = modulus;
            } else if (headerRawResidueModulus != modulus) {
                headerRawResidueModulusConsistent = false;
            }

            headerRawResidues.push_back(static_cast<int>(diagnostic.rawSymbol % modulus));
            headerDecodedSymbols.push_back(static_cast<unsigned short>(diagnostic.decoderSymbol));
        }

        if (!headerRawResidueModulusConsistent) {
            headerRawResidueModulus = 0U;
        }

        int headerRawResidueMode = -1;
        unsigned int headerRawResidueModeCount = 0U;

        for (int residue : headerRawResidues)
        {
            const unsigned int count = static_cast<unsigned int>(
                std::count(headerRawResidues.begin(), headerRawResidues.end(), residue));

            if (count > headerRawResidueModeCount)
            {
                headerRawResidueModeCount = count;
                headerRawResidueMode = residue;
            }
        }

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

        auto restoreLoRaState = [this, &msgBytes](const LoRaDecodeState& state) {
            msgBytes = state.bytes;
            m_hasCRC = state.hasCRC;
            m_nbParityBits = state.nbParityBits;
            m_packetLength = state.packetLength;
            m_nbSymbols = state.nbSymbols;
            m_nbCodewords = state.nbCodewords;
            m_earlyEOM = state.earlyEOM;
            m_headerParityStatus = state.headerParityStatus;
            m_headerCRCStatus = state.headerCRCStatus;
            m_payloadParityStatus = state.payloadParityStatus;
            m_payloadCRCStatus = state.payloadCRCStatus;
        };

        auto decodeSymbolsWithTrace = [this](
            const std::vector<unsigned short>& symbols,
            QByteArray& bytes,
            MeshtasticDemodDecoderLoRa::DecodeTrace& trace)
        {
            trace = MeshtasticDemodDecoderLoRa::DecodeTrace();

            if (m_nbSymbolBits >= 5)
            {
                const unsigned int headerNbSymbolBits =
                    (m_hasHeader && (m_spreadFactor > 2U))
                        ? (m_spreadFactor - 2U)
                        : m_nbSymbolBits;

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
                    m_payloadCRCStatus,
                    &trace
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
        };

        auto makeAttempt = [](const QString& id, int headerDelta, int payloadDelta) {
            MeshtasticDemodMsg::DecodeAttemptDiagnostic attempt;
            attempt.attemptId = id;
            attempt.headerDelta = headerDelta;
            attempt.payloadDelta = payloadDelta;
            attempt.stopReason = QStringLiteral("not_candidate");
            return attempt;
        };

        auto populateAttempt = [](
            MeshtasticDemodMsg::DecodeAttemptDiagnostic& attempt,
            const LoRaDecodeState& state,
            const MeshtasticDemodDecoderLoRa::DecodeTrace& trace)
        {
            attempt.executed = true;
            attempt.headerCRCComputed = trace.headerCRCComputed;
            attempt.headerCRCStatus = state.headerCRCStatus;
            attempt.hasCRC = state.hasCRC;
            attempt.packetLength = state.packetLength;
            attempt.nbParityBits = state.nbParityBits;
            attempt.earlyEOM = state.earlyEOM;
            attempt.payloadDecodeCompleted = trace.payloadDecodeCompleted;
            attempt.payloadParityStatus = state.payloadParityStatus;
            attempt.payloadCRCComputed = trace.payloadCRCComputed;
            attempt.payloadCRCStatus = state.payloadCRCStatus;
            attempt.crcDataOffset = trace.crcDataOffset;
            attempt.crc16ByteCount = trace.crc16ByteCount;
            attempt.crcTailByte0Offset = trace.crcTailByte0Offset;
            attempt.crcTailByte1Offset = trace.crcTailByte1Offset;
            attempt.receivedCRCOffset = trace.receivedCRCOffset;
            attempt.calculatedCRC = trace.calculatedCRC;
            attempt.receivedCRC = trace.receivedCRC;
            attempt.bytes = state.bytes;

            if (trace.headerCRCComputed && !state.headerCRCStatus) {
                attempt.stopReason = QStringLiteral("header_crc_fail");
            } else if (state.earlyEOM) {
                attempt.stopReason = QStringLiteral("early_eom");
            } else if (!state.hasCRC) {
                attempt.stopReason = QStringLiteral("no_payload_crc");
            } else if (trace.payloadCRCComputed && state.payloadCRCStatus) {
                attempt.stopReason = QStringLiteral("payload_crc_pass");
            } else if (trace.payloadCRCComputed) {
                attempt.stopReason = QStringLiteral("payload_crc_fail");
            } else {
                attempt.stopReason = QStringLiteral("early_eom");
            }
        };

        QString decodePath = QStringLiteral("failed");
        QByteArray decodeSoftBytes;
        QByteArray decodeHardBytes;
        QByteArray decodeMinus1Bytes;
        QByteArray decodePlus1Bytes;

        MeshtasticDemodMsg::DecodeAttemptDiagnostic baseSoftAttempt =
            makeAttempt(QStringLiteral("base_soft"), 0, 0);
        MeshtasticDemodMsg::DecodeAttemptDiagnostic baseHardAttempt =
            makeAttempt(QStringLiteral("base_hard"), 0, 0);
        MeshtasticDemodMsg::DecodeAttemptDiagnostic wholeMinus1Attempt =
            makeAttempt(QStringLiteral("whole_minus1"), -1, -1);
        MeshtasticDemodMsg::DecodeAttemptDiagnostic wholePlus1Attempt =
            makeAttempt(QStringLiteral("whole_plus1"), 1, 1);
        MeshtasticDemodMsg::DecodeAttemptDiagnostic splitR2Attempt =
            makeAttempt(QStringLiteral("split_r2"), 0, -1);

        // Record the second decode that changes FFT bins before LoRa symbol conversion.
        MeshtasticDemodMsg::DecodeAttemptDiagnostic rawFftAttempt =
            makeAttempt(QStringLiteral("raw_fft_shift"), 0, 0);

        if (canSoftDecode)
        {
            const unsigned int headerNbSymbolBits =
                (m_hasHeader && (m_spreadFactor > 2U))
                    ? (m_spreadFactor - 2U)
                    : m_nbSymbolBits;
            MeshtasticDemodDecoderLoRa::DecodeTrace softTrace;

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
                m_payloadCRCStatus,
                &softTrace
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
            decodeSoftBytes = softState.bytes;
            populateAttempt(baseSoftAttempt, softState, softTrace);

            if (m_hasCRC && !m_payloadCRCStatus)
            {
                QByteArray hardBytes;
                MeshtasticDemodDecoderLoRa::DecodeTrace hardTrace;
                decodeSymbolsWithTrace(msg.getSymbols(), hardBytes, hardTrace);
                const LoRaDecodeState hardState = captureLoRaState(hardBytes);
                decodeHardBytes = hardState.bytes;
                populateAttempt(baseHardAttempt, hardState, hardTrace);

                if (hardState.payloadCRCStatus) {
                    restoreLoRaState(hardState);
                    decodePath = QStringLiteral("hard");
                } else {
                    restoreLoRaState(softState);
                }
            }
            else if (m_payloadCRCStatus)
            {
                decodePath = QStringLiteral("soft");
            }
        }
        else
        {
            QByteArray hardBytes;
            MeshtasticDemodDecoderLoRa::DecodeTrace hardTrace;
            decodeSymbolsWithTrace(msg.getSymbols(), hardBytes, hardTrace);
            const LoRaDecodeState hardState = captureLoRaState(hardBytes);
            decodeHardBytes = hardState.bytes;
            populateAttempt(baseHardAttempt, hardState, hardTrace);
            msgBytes = hardState.bytes;

            if (m_payloadCRCStatus) {
                decodePath = QStringLiteral("hard");
            }
        }

        const LoRaDecodeState preRetryState = captureLoRaState(msgBytes);
        const bool headerRawResidueUniform =
            (headerRawResidues.size() == 8U)
            && std::all_of(
                headerRawResidues.begin() + 1,
                headerRawResidues.end(),
                [&](int residue) { return residue == headerRawResidues.front(); });
        // Mark the complete 8-symbol LoRa header when every FFT peak is one bin low.
        const bool loRa8SymbolHeaderOneBinLow =
            headerRawResidueUniform
            && (headerRawResidueModulus == 4U)
            && (headerRawResidueMode == 0);

        // Mark the complete 8-symbol LoRa header when every FFT peak is one bin high.
        const bool loRa8SymbolHeaderOneBinHigh =
            headerRawResidueUniform
            && (headerRawResidueModulus == 4U)
            && (headerRawResidueMode == 2);

        // Reuse the same 8-symbol LoRa header test for the existing -1 payload recovery.
        const bool rawResidue2 = loRa8SymbolHeaderOneBinHigh;

        auto makePayloadShiftedSymbols = [&](int delta)
        {
            std::vector<unsigned short> shifted = msg.getSymbols();
            const unsigned int payloadMod = 1U << std::max(1U, m_nbSymbolBits);

            for (size_t i = 8U; i < shifted.size(); ++i)
            {
                const int symbol = static_cast<int>(shifted[i]);
                const int shiftedSymbol = (symbol + delta) % static_cast<int>(payloadMod);
                shifted[i] = static_cast<unsigned short>(
                    shiftedSymbol < 0 ? shiftedSymbol + static_cast<int>(payloadMod) : shiftedSymbol);
            }

            return shifted;
        };

        bool splitR2Recovered = false;

        // DECODER BEHAVIOR CHANGE: promote the measured raw-residue-2 recovery
        // from diagnostic-only to production. Keep the validated base header
        // unchanged and shift only payload symbols by -1 before CRC acceptance.
        if (rawResidue2
            && preRetryState.headerCRCStatus
            && preRetryState.hasCRC
            && !preRetryState.payloadCRCStatus
            && (m_spreadFactor >= 5U))
        {
            std::vector<unsigned short> shifted = makePayloadShiftedSymbols(-1);

            QByteArray splitBytes;
            MeshtasticDemodDecoderLoRa::DecodeTrace splitTrace;
            decodeSymbolsWithTrace(shifted, splitBytes, splitTrace);
            const LoRaDecodeState splitState = captureLoRaState(splitBytes);
            populateAttempt(splitR2Attempt, splitState, splitTrace);

            if (splitState.headerCRCStatus
                && splitState.hasCRC
                && splitTrace.payloadCRCComputed
                && splitState.payloadCRCStatus)
            {
                restoreLoRaState(splitState);
                decodePath = QStringLiteral("split_r2");
                splitR2Recovered = true;
            }
            else
            {
                restoreLoRaState(preRetryState);
            }
        }

        // DECODER BEHAVIOR CHANGE: an invalid base header must not veto recovery
        // using its unvalidated hasCRC bit. Allow whole-shift retries after any
        // failed payload decode; each candidate must validate its own fresh header.
        const bool wholeRetryGateOpen =
            !splitR2Recovered
            && !preRetryState.payloadCRCStatus
            && (m_spreadFactor >= 5U);

        if (wholeRetryGateOpen)
        {
            const unsigned int headerNbSymbolBits = (m_hasHeader && (m_spreadFactor > 2U))
                ? (m_spreadFactor - 2U)
                : m_nbSymbolBits;
            bool recovered = false;

            for (int delta : {-1, 1})
            {
                // DECODER BEHAVIOR CHANGE: every whole-shift candidate starts
                // from the same pre-retry decoder state so the -1 probe cannot
                // contaminate the subsequent +1 production candidate.
                restoreLoRaState(preRetryState);

                MeshtasticDemodMsg::DecodeAttemptDiagnostic& attempt =
                    (delta == -1) ? wholeMinus1Attempt : wholePlus1Attempt;
                std::vector<unsigned short> shifted = msg.getSymbols();

                for (size_t i = 0; i < shifted.size(); i++)
                {
                    const bool isHeader = m_hasHeader && (i < 8U);
                    const unsigned int bits = isHeader ? headerNbSymbolBits : m_nbSymbolBits;
                    const unsigned int mod = 1U << std::max(1U, bits);
                    const int symbol = static_cast<int>(shifted[i]);
                    const int shiftedSymbol = (symbol + delta) % static_cast<int>(mod);
                    shifted[i] = static_cast<unsigned short>(
                        shiftedSymbol < 0 ? shiftedSymbol + static_cast<int>(mod) : shiftedSymbol);
                }

                QByteArray shiftedBytes;
                MeshtasticDemodDecoderLoRa::DecodeTrace shiftedTrace;
                decodeSymbolsWithTrace(shifted, shiftedBytes, shiftedTrace);
                const LoRaDecodeState shiftedState = captureLoRaState(shiftedBytes);
                populateAttempt(attempt, shiftedState, shiftedTrace);

                if (delta == -1) {
                    decodeMinus1Bytes = shiftedState.bytes;
                } else {
                    decodePlus1Bytes = shiftedState.bytes;
                }

                const bool candidatePassed =
                    shiftedTrace.headerCRCComputed
                    && shiftedState.headerCRCStatus
                    && shiftedState.hasCRC
                    && shiftedTrace.payloadCRCComputed
                    && shiftedState.payloadCRCStatus;

                // DECODER BEHAVIOR CHANGE: keep whole -1 as an instrumented
                // probe, but never allow it to accept a frame. Only the +1
                // candidate remains eligible for production recovery.
                if ((delta == 1) && candidatePassed)
                {
                    restoreLoRaState(shiftedState);
                    decodePath = QStringLiteral("plus1_bin");
                    recovered = true;
                    break;
                }
            }

            if (!recovered) {
                restoreLoRaState(preRetryState);
            }
        }
        else
        {
            const QString gateReason = splitR2Recovered
                ? QStringLiteral("prior_attempt_passed")
                : QStringLiteral("not_candidate");
            wholeMinus1Attempt.stopReason = gateReason;
            wholePlus1Attempt.stopReason = gateReason;
        }

        // Everything below is diagnostic-only. Preserve the production-selected
        // state and run missing alternatives without changing decodePath or output.
        const LoRaDecodeState acceptedState = captureLoRaState(msgBytes);

        // Select the FFT-bin change from the complete 8-symbol LoRa header.
        int rawFftBinDelta = 0;
        if (loRa8SymbolHeaderOneBinLow) {
            rawFftBinDelta = +1;
        } else if (loRa8SymbolHeaderOneBinHigh) {
            rawFftBinDelta = -1;
        }
        rawFftAttempt.rawFftBinDelta = rawFftBinDelta;

        // The 8-symbol LoRa header FFT peak bins normally fall on bin indexes 1,5,9,... modulo 4.
        // If all 8 LoRa header FFT peak bins fall one bin low, decode all symbols in the message with FFT bin +1.
        // If all 8 LoRa header FFT peak bins fall one bin high, decode all symbols in the message with FFT bin -1.
        // This second decode only records its result; it does not change the message returned by the current decoder.
        if ((rawFftBinDelta != 0)
            && m_hasHeader
            && (m_spreadFactor >= 5U)
            && (msg.getRawFftBins().size() == msg.getSymbols().size()))
        {
            const std::vector<unsigned int>& rawFftBins = msg.getRawFftBins();
            const unsigned int fftBinCount = 1U << m_spreadFactor;
            std::vector<unsigned short> remappedSymbols;
            remappedSymbols.reserve(rawFftBins.size());

            // Rebuild every LoRa symbol after changing its original FFT peak bin.
            for (size_t i = 0; i < rawFftBins.size(); ++i)
            {
                const int correctedBin =
                    (static_cast<int>(rawFftBins[i])
                        + rawFftBinDelta
                        + static_cast<int>(fftBinCount))
                    % static_cast<int>(fftBinCount);
                const bool loRaHeaderSymbol = i < 8U;
                unsigned int spread = 1U << m_deBits;

                if (loRaHeaderSymbol && (m_deBits < 2U)) {
                    spread <<= (2U - m_deBits);
                }

                const unsigned int shiftedBin =
                    (static_cast<unsigned int>(correctedBin)
                        + fftBinCount - 1U)
                    % fftBinCount;
                remappedSymbols.push_back(
                    static_cast<unsigned short>(shiftedBin / std::max(1U, spread)));
            }

            // Decode the rebuilt symbols from the same state used before current recovery attempts.
            restoreLoRaState(preRetryState);
            QByteArray rawFftBytes;
            MeshtasticDemodDecoderLoRa::DecodeTrace rawFftTrace;
            decodeSymbolsWithTrace(remappedSymbols, rawFftBytes, rawFftTrace);
            const LoRaDecodeState rawFftState = captureLoRaState(rawFftBytes);
            populateAttempt(rawFftAttempt, rawFftState, rawFftTrace);

            // Restore the production-selected result after the parallel decode finishes.
            restoreLoRaState(acceptedState);
        }

        const bool diagnosticEligible =
            m_hasHeader
            && (msg.getSymbols().size() >= 8U)
            && (m_spreadFactor >= 5U);

        if (diagnosticEligible)
        {
            if (!baseHardAttempt.executed)
            {
                restoreLoRaState(acceptedState);
                QByteArray hardBytes;
                MeshtasticDemodDecoderLoRa::DecodeTrace hardTrace;
                decodeSymbolsWithTrace(msg.getSymbols(), hardBytes, hardTrace);
                const LoRaDecodeState hardState = captureLoRaState(hardBytes);
                decodeHardBytes = hardState.bytes;
                populateAttempt(baseHardAttempt, hardState, hardTrace);
                restoreLoRaState(acceptedState);
            }

            const unsigned int headerNbSymbolBits =
                (m_hasHeader && (m_spreadFactor > 2U))
                    ? (m_spreadFactor - 2U)
                    : m_nbSymbolBits;

            auto runWholeShiftDiagnostic = [&](int delta,
                                               MeshtasticDemodMsg::DecodeAttemptDiagnostic& attempt,
                                               QByteArray& diagnosticBytes)
            {
                if (attempt.executed) {
                    return;
                }

                restoreLoRaState(acceptedState);
                std::vector<unsigned short> shifted = msg.getSymbols();

                for (size_t i = 0; i < shifted.size(); ++i)
                {
                    const bool isHeader = m_hasHeader && (i < 8U);
                    const unsigned int bits = isHeader ? headerNbSymbolBits : m_nbSymbolBits;
                    const unsigned int mod = 1U << std::max(1U, bits);
                    const int symbol = static_cast<int>(shifted[i]);
                    const int shiftedSymbol = (symbol + delta) % static_cast<int>(mod);
                    shifted[i] = static_cast<unsigned short>(
                        shiftedSymbol < 0 ? shiftedSymbol + static_cast<int>(mod) : shiftedSymbol);
                }

                QByteArray shiftedBytes;
                MeshtasticDemodDecoderLoRa::DecodeTrace shiftedTrace;
                decodeSymbolsWithTrace(shifted, shiftedBytes, shiftedTrace);
                const LoRaDecodeState shiftedState = captureLoRaState(shiftedBytes);
                diagnosticBytes = shiftedState.bytes;
                populateAttempt(attempt, shiftedState, shiftedTrace);
                restoreLoRaState(acceptedState);
            };

            // Run whichever whole-frame alternatives production did not execute.
            // This explicitly measures gate-closed and prior-attempt-skipped cases.
            runWholeShiftDiagnostic(-1, wholeMinus1Attempt, decodeMinus1Bytes);
            runWholeShiftDiagnostic(1, wholePlus1Attempt, decodePlus1Bytes);

            // If production did not run split_r2, retain the unconditional
            // diagnostic control so cross-residue specificity remains measured.
            if (!splitR2Attempt.executed)
            {
                restoreLoRaState(acceptedState);
                std::vector<unsigned short> shifted = makePayloadShiftedSymbols(-1);

                QByteArray splitBytes;
                MeshtasticDemodDecoderLoRa::DecodeTrace splitTrace;
                decodeSymbolsWithTrace(shifted, splitBytes, splitTrace);
                const LoRaDecodeState splitState = captureLoRaState(splitBytes);
                populateAttempt(splitR2Attempt, splitState, splitTrace);
                restoreLoRaState(acceptedState);
            }
        }
        else
        {
            if (!baseHardAttempt.executed) {
                baseHardAttempt.stopReason = QStringLiteral("not_applicable");
            }
            if (!wholeMinus1Attempt.executed) {
                wholeMinus1Attempt.stopReason = QStringLiteral("not_applicable");
            }
            if (!wholePlus1Attempt.executed) {
                wholePlus1Attempt.stopReason = QStringLiteral("not_applicable");
            }
            splitR2Attempt.stopReason = QStringLiteral("not_applicable");
        }

        // Record the parallel FFT-bin decode with the existing decoder attempts.
        std::vector<MeshtasticDemodMsg::DecodeAttemptDiagnostic> decodeAttemptDiagnostics {
            baseSoftAttempt,
            baseHardAttempt,
            wholeMinus1Attempt,
            wholePlus1Attempt,
            splitR2Attempt,
            rawFftAttempt
        };

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
            outputMsg->setFrameId(msg.getFrameId());
            outputMsg->setSyncWord(msgSyncWord);
            outputMsg->setSignalDb(msgSignalDb);
            outputMsg->setNoiseDb(msgNoiseDb);
            outputMsg->setCfoHz(msgCfoHz);
            outputMsg->setSfoPpm(msgSfoPpm);
            outputMsg->setFftMarginMinDb(fftMarginMinDb);
            outputMsg->setFftMarginAvgDb(fftMarginAvgDb);
            outputMsg->setFftMarginLt1Db(fftMarginLt1Db);
            outputMsg->setFftMarginLt3Db(fftMarginLt3Db);
            outputMsg->setTransientSummary(
                fftBestFractionMin,
                fftBestFractionWorstSymbol,
                fftTotalPowerPeakToMedianDb,
                fftTotalPowerPeakSymbol,
                samplePeakToRmsMaxDb,
                samplePeakToRmsWorstSymbol,
                sampleMeanPowerPeakToMedianDb,
                sampleMeanPowerPeakSymbol
            );
            outputMsg->setProductionWholeRetryGateOpen(wholeRetryGateOpen);
            outputMsg->setDecodeAttemptDiagnostics(decodeAttemptDiagnostics);
            outputMsg->setHeaderRawResidueMetadata(
                headerRawResidues,
                headerDecodedSymbols,
                headerRawResidueModulus,
                headerRawResidueMode
            );
            outputMsg->setBaseRetryGateState(
                preRetryState.hasCRC,
                preRetryState.headerCRCStatus
            );

            const bool splitPassed =
                splitR2Attempt.executed
                && splitR2Attempt.payloadCRCComputed
                && splitR2Attempt.payloadCRCStatus;
            const bool splitUnexpectedPass = splitPassed && !rawResidue2;
            const bool splitUnexpectedFail =
                rawResidue2
                && splitR2Attempt.executed
                && (!splitR2Attempt.payloadCRCComputed || !splitR2Attempt.payloadCRCStatus);
            const bool legacyHasCRCGateWouldHaveClosed =
                wholeRetryGateOpen
                && !preRetryState.hasCRC;
            outputMsg->setLegacyHasCRCGateWouldHaveClosed(
                legacyHasCRCGateWouldHaveClosed
            );
            // Keep full symbol details whenever the parallel FFT-bin decode runs.
            const bool verboseDecoderDiagnostics =
                (decodePath == QStringLiteral("failed"))
                || (decodePath == QStringLiteral("split_r2"))
                || (decodePath == QStringLiteral("minus1_bin"))
                || (decodePath == QStringLiteral("plus1_bin"))
                || rawFftAttempt.executed
                || splitUnexpectedPass
                || splitUnexpectedFail
                || legacyHasCRCGateWouldHaveClosed;

            outputMsg->setVerboseDecoderDiagnostics(verboseDecoderDiagnostics);

            if (verboseDecoderDiagnostics)
            {
                outputMsg->setFftPeakDiagnostics(fftPeakDiagnostics);
                outputMsg->setSymbolMappingDiagnostics(msg.getSymbolMappingDiagnostics());
                outputMsg->setTransientDiagnostics(msg.getTransientDiagnostics());
            }

            outputMsg->setHeaderLockDiagnostic(
                msg.getHeaderLockDiagnosticValid(),
                msg.getHeaderLockOffset(),
                msg.getHeaderLockDelta()
            );
            outputMsg->setDecodePath(decodePath);
            if (m_hasCRC && !m_payloadCRCStatus)
            {
                outputMsg->setDecodeSoftBytes(decodeSoftBytes);
                outputMsg->setDecodeHardBytes(decodeHardBytes);
                outputMsg->setDecodeMinus1Bytes(decodeMinus1Bytes);
                outputMsg->setDecodePlus1Bytes(decodePlus1Bytes);
            }
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

    while ((message = m_inputMessageQueue.pop()) != nullptr)
    {
        if (handleMessage(*message)) {
            delete message;
        }
    }
}
