///////////////////////////////////////////////////////////////////////////////////
// Copyright (C) 2019-2020 Edouard Griffiths, F4EXB <f4exb06@gmail.com>          //
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
#include <QDateTime>
#include <QDebug>
#include <QStringList>
#include <cstring>
#include <QtEndian>
#include <QStandardPaths>
#include <QFile>
#include <QDir>
#include <QUuid>
#include <stdio.h>
#include <algorithm>
#include <limits>
#include <cmath>
#include <vector>
#include <thread>
#include <utility>

#include "dsp/dsptypes.h"
#include "dsp/basebandsamplesink.h"
#include "dsp/dspengine.h"
#include "dsp/fftfactory.h"
#include "dsp/fftengine.h"
#include "util/db.h"

#include "meshtasticdemodmsg.h"
#include "meshtasticdemoddecoderlora.h"
#include "meshtasticdemodsink.h"

MeshtasticDemodSink::MeshtasticDemodSink() :
    m_decodeMsg(nullptr),
    m_decoderMsgQueue(nullptr),
    m_fftSequence(-1),
    m_downChirps(nullptr),
    m_upChirps(nullptr),
    m_spectrumLine(nullptr),
    m_magsqMax(0.0),
    m_headerLocked(false),
    m_expectedSymbols(0),
    m_waitHeaderFeedback(false),
    m_headerFeedbackWaitSteps(0U),
    m_loRaFrameId(0U),
    m_tempIqDeviceCenterFrequency(0),
    m_tempIqInputFrequencyOffset(0),
    m_tempIqChannelFrequencyOffset(0),
    m_tempIqBandwidth(0),
    m_tempIqNbSymbols(0U),
    m_tempIqCaptureEnabled(false),
    m_tempIqSampleRate(0U),
    m_tempIqChannelSamplesPerSymbol(1U),
    m_tempIqPreRollLimit(1U),
    m_tempIqMaxCaptureSamples(1U),
    m_osFactor(MeshtasticDemodSettings::oversampling > 0 ? MeshtasticDemodSettings::oversampling : 1),
    m_osCenterPhase((MeshtasticDemodSettings::oversampling > 1 ? MeshtasticDemodSettings::oversampling / 2 : 0)),
    m_osCounter(0),
    m_loRaState(LoRaStateDetect),
    m_loRaSyncState(LoRaSyncNetId1),
    m_loRaSymbolCnt(1),
    m_loRaBinIdx(0),
    m_loRaKHat(0),
    m_loRaDownVal(0),
    m_loRaCFOInt(0),
    m_loRaNetIdOff(0),
    m_loRaAdditionalUpchirps(0),
    m_loRaUpSymbToUse(0),
    m_loRaRequiredUpchirps(0),
    m_loRaSymbolSpan(0),
    m_loRaFrameSymbolCount(0),
    m_loRaCFOFrac(0.0f),
    m_loRaSTOFrac(0.0f),
    m_loRaSFOHat(0.0f),
    m_loRaSFOCum(0.0f),
    m_loRaCFOSTOEstimated(false),
    m_loRaReceivedHeader(false),
    m_loRaOneSymbolOff(false),
    m_spectrumSink(nullptr),
    m_spectrumBuffer(nullptr)
{
    m_demodActive = false;
	m_bandwidth = MeshtasticDemodSettings::bandwidths[0];
	m_channelSampleRate = 96000;
	m_channelFrequencyOffset = 0;
    m_deviceCenterFrequency = 0;
	m_nco.setFreq(m_channelFrequencyOffset, m_channelSampleRate);
    {
        const double loggedCutoffHz = static_cast<double>(m_bandwidth) / 1.9;
        qInfo().noquote()
            << QStringLiteral("MESHTASTIC_INTERP_CREATE ts=%1 frame=%2 reason=ctor channel_sr=%3 requested_bw=%4 previous_bw=%5 force=1 old_cutoff_hz=%6 new_cutoff_hz=%7")
                .arg(QDateTime::currentDateTime().toString(Qt::ISODateWithMs))
                .arg(m_loRaFrameId)
                .arg(m_channelSampleRate)
                .arg(m_bandwidth)
                .arg(m_bandwidth)
                .arg(loggedCutoffHz, 0, 'f', 3)
                .arg(loggedCutoffHz, 0, 'f', 3);
        // Preserve the original float cutoff expression exactly; logging must
        // not perturb the filter under test.
        m_interpolator.create(16, m_channelSampleRate, m_bandwidth / 1.9f);
    }
    m_interpolatorDistance = (Real) m_channelSampleRate / (Real) m_bandwidth;
    m_sampleDistanceRemain = 0;
    const unsigned int ctorConfiguredPreamble = m_settings.m_preambleChirps > 0U
        ? m_settings.m_preambleChirps
        : m_minRequiredPreambleChirps;
    const unsigned int ctorTargetRequired = ctorConfiguredPreamble > 3U
        ? (ctorConfiguredPreamble - 3U)
        : m_minRequiredPreambleChirps;
    m_requiredPreambleChirps = std::max(
        m_minRequiredPreambleChirps,
        std::min(ctorTargetRequired, m_maxRequiredPreambleChirps)
    );
    m_fftInterpolation = m_loRaFFTInterpolation;

    initSF(m_settings.m_spreadFactor, m_settings.m_deBits);
}

MeshtasticDemodSink::~MeshtasticDemodSink()
{
    FFTFactory *fftFactory = DSPEngine::instance()->getFFTFactory();

    if (m_fftSequence >= 0)
    {
        fftFactory->releaseEngine(m_interpolatedFFTLength, false, m_fftSequence);
    }

    delete[] m_downChirps;
    delete[] m_upChirps;
    delete[] m_spectrumBuffer;
    delete[] m_spectrumLine;
}

void MeshtasticDemodSink::initSF(unsigned int sf, unsigned int deBits)
{
    if (m_downChirps) {
        delete[] m_downChirps;
    }
    if (m_upChirps) {
        delete[] m_upChirps;
    }
    if (m_spectrumBuffer) {
        delete[] m_spectrumBuffer;
    }
    if (m_spectrumLine) {
        delete[] m_spectrumLine;
    }

    FFTFactory *fftFactory = DSPEngine::instance()->getFFTFactory();

    if (m_fftSequence >= 0) {
        fftFactory->releaseEngine(m_interpolatedFFTLength, false, m_fftSequence);
    }

    m_nbSymbols = 1 << sf;
    m_nbSymbolsEff = 1 << (sf - deBits);
    m_fftLength = m_nbSymbols;
    m_interpolatedFFTLength = m_fftInterpolation*m_fftLength;
    m_fftSequence = fftFactory->getEngine(m_interpolatedFFTLength, false, &m_fft);
    m_downChirps = new Complex[2*m_nbSymbols]; // Each table is 2 chirps long to allow processing from arbitrary offsets.
    m_upChirps = new Complex[2*m_nbSymbols];
    m_spectrumBuffer = new Complex[m_nbSymbols];
    m_spectrumLine = new Complex[m_nbSymbols];
    std::fill(m_spectrumLine, m_spectrumLine+m_nbSymbols, Complex(std::polar(1e-6*SDR_RX_SCALED, 0.0)));
    m_loRaSymbolSpan = m_nbSymbols * m_osFactor;
    m_loRaRequiredUpchirps = m_requiredPreambleChirps;
    m_loRaUpSymbToUse = (m_loRaRequiredUpchirps > 0U) ? static_cast<int>(m_loRaRequiredUpchirps - 1U) : 0;
    m_loRaInDown.assign(m_nbSymbols, Complex{0.0f, 0.0f});
    m_loRaPreambleRaw.assign(m_nbSymbols * m_loRaRequiredUpchirps, Complex{0.0f, 0.0f});
    m_loRaPreambleRawUp.assign((m_settings.m_preambleChirps + 3U) * m_loRaSymbolSpan, Complex{0.0f, 0.0f});
    m_loRaPreambleUpchirps.assign(m_nbSymbols * m_loRaRequiredUpchirps, Complex{0.0f, 0.0f});
    m_loRaCFOFracCorrec.assign(m_nbSymbols, Complex{1.0f, 0.0f});
    m_loRaPayloadDownchirp.assign(m_nbSymbols, Complex{1.0f, 0.0f});
    m_loRaSymbCorr.assign(m_nbSymbols, Complex{0.0f, 0.0f});
    m_loRaNetIdSamp.assign((m_loRaSymbolSpan * 5U) / 2U + m_loRaSymbolSpan, Complex{0.0f, 0.0f});
    m_loRaAdditionalSymbolSamp.assign(m_loRaSymbolSpan * 2U, Complex{0.0f, 0.0f});
    m_loRaPreambleVals.assign(m_loRaRequiredUpchirps, 0);
    m_loRaNetIds.assign(2, 0);
    m_loRaSampleFifo.clear();
    m_loRaState = LoRaStateDetect;
    m_loRaSyncState = LoRaSyncNetId1;
    m_loRaSymbolCnt = 1;
    m_loRaBinIdx = 0;
    m_loRaKHat = 0;
    m_loRaAdditionalUpchirps = 0;
    m_loRaCFOSTOEstimated = false;
    m_loRaReceivedHeader = false;
    m_loRaFrameSymbolCount = 0;
    updateTempIqCaptureGeometry();

    // Canonical gr-lora_sdr reference chirps (utilities::build_ref_chirps, id=0, os_factor=1).
    for (unsigned int i = 0; i < m_fftLength; i++)
    {
        const double n = static_cast<double>(i);
        const double N = static_cast<double>(m_nbSymbols);
        const double phase = 2.0 * M_PI * ((n * n) / (2.0 * N) - 0.5 * n);
        m_upChirps[i] = Complex(std::cos(phase), std::sin(phase));
        m_downChirps[i] = std::conj(m_upChirps[i]);
    }

    // Duplicate table to allow processing from arbitrary offsets
    std::copy(m_downChirps, m_downChirps+m_fftLength, m_downChirps+m_fftLength);
    std::copy(m_upChirps, m_upChirps+m_fftLength, m_upChirps+m_fftLength);
}

void MeshtasticDemodSink::feed(const SampleVector::const_iterator& begin, const SampleVector::const_iterator& end)
{
	Complex ci;

	for (SampleVector::const_iterator it = begin; it < end; ++it)
	{
		Complex c(it->real() / SDR_RX_SCALEF, it->imag() / SDR_RX_SCALEF);
		c *= m_nco.nextIQ();

        // Capture channel-rate IQ before the interpolator so replay can exercise
        // historical anti-alias cutoff behavior, including 651ca8c0a.
        updateTempIqCapture(c);

		if (m_interpolator.decimate(&m_sampleDistanceRemain, c, &ci))
		{
            if (MeshtasticDemodSettings::m_codingScheme == MeshtasticDemodSettings::CodingLoRa)
            {
                processSampleLoRa(ci);
            }
            else if (m_osFactor <= 1U)
            {
                processSampleLoRa(ci);
            }
            else
            {
                if ((m_osCounter % m_osFactor) == m_osCenterPhase) {
                    processSampleLoRa(ci);
                }
                m_osCounter++;
            }
			m_sampleDistanceRemain += m_interpolatorDistance;
		}
	}
}

void MeshtasticDemodSink::reset()
{
    resetLoRaFrameSync();
    m_headerLocked = false;
    m_expectedSymbols = 0;
    m_waitHeaderFeedback = false;
    m_headerFeedbackWaitSteps = 0;
    m_osCounter = 0;
}

unsigned int MeshtasticDemodSink::argmax(
    const Complex *fftBins,
    unsigned int fftMult,
    unsigned int fftLength,
    double& magsqMax,
    double& magsqTotal,
    Complex *specBuffer,
    unsigned int specDecim)
{
    magsqMax = 0.0;
    magsqTotal = 0.0;
    unsigned int imax = 0;
    double magSum = 0.0;
    std::vector<double> spectrumBucketPowers;

    if (specBuffer) {
        spectrumBucketPowers.reserve((fftMult * fftLength) / std::max(1U, specDecim));
    }

    for (unsigned int i = 0; i < fftMult*fftLength; i++)
    {
        double magsq = std::norm(fftBins[i]);
        magsqTotal += magsq;

        if (magsq > magsqMax)
        {
            imax = i;
            magsqMax = magsq;
        }

        if (specBuffer)
        {
            magSum += magsq;

            if (i % specDecim == specDecim - 1)
            {
                spectrumBucketPowers.push_back(magSum);
                magSum = 0.0;
            }
        }
    }

    const double magsqAvgRaw = magsqTotal / static_cast<double>(fftMult * fftLength);
    magsqTotal = magsqAvgRaw;

    if (specBuffer && !spectrumBucketPowers.empty())
    {
        const double noisePerBucket = magsqAvgRaw * static_cast<double>(std::max(1U, specDecim));
        const double floorCut = noisePerBucket * 1.05; // suppress steady floor
        const double boost = 12.0;                     // emphasize peaks over residual floor

        for (size_t i = 0; i < spectrumBucketPowers.size(); ++i)
        {
            const double enhancedPower = std::max(0.0, spectrumBucketPowers[i] - floorCut) * boost;
            const double specAmp = std::sqrt(enhancedPower) * static_cast<double>(m_nbSymbols);
            specBuffer[i] = Complex(std::polar(specAmp, 0.0));
        }
    }

    return imax;
}

unsigned int MeshtasticDemodSink::evalSymbol(unsigned int rawSymbol, bool headerSymbol)
{
    unsigned int spread = m_fftInterpolation * (1U << m_settings.m_deBits);
    const unsigned int symbolBins = m_fftInterpolation * m_nbSymbols;

    if (symbolBins == 0U) {
        return rawSymbol;
    }

    // In gr-lora_sdr, explicit-header symbols are always reduced by 2 extra bits
    // (sf_app = sf-2), independently of payload LDRO selection.
    if (headerSymbol)
    {
        const int de = m_settings.m_deBits;
        if (de < 2) {
            spread <<= (2 - de);
        }
    }

    // Match gr-lora_sdr hard-decoding symbol mapping:
    //   s = mod(raw_bin - 1, 2^SF * os_factor) / (os_factor * 2^DE)
    const unsigned int shifted = (rawSymbol + symbolBins - 1U) % symbolBins;

    if (spread == 0U) {
        return shifted;
    }

    return shifted / spread;
}

void MeshtasticDemodSink::tryHeaderLock()
{
    if (!m_decodeMsg) {
        return;
    }

    const std::vector<unsigned short>& symbols = m_decodeMsg->getSymbols();

    if (symbols.size() < 8) {
        return;
    }

    const unsigned int sf = m_settings.m_spreadFactor;

    if (sf < 7) {
        return;
    }

    const unsigned int headerNbSymbolBits = sf - 2U;

    if (headerNbSymbolBits < 5) {
        return;
    }

    const unsigned int maxOffset = std::min<unsigned int>(2U, static_cast<unsigned int>(symbols.size()) - 8U);
    const unsigned int headerSymbolMod = 1U << headerNbSymbolBits;

    for (unsigned int offset = 0U; offset <= maxOffset; offset++)
    {
        const std::vector<unsigned short> baseHeaderSlice(symbols.begin() + offset, symbols.begin() + offset + 8U);

        for (int delta = -2; delta <= 2; delta++)
        {
            std::vector<unsigned short> headerSlice(baseHeaderSlice);

            if (delta != 0)
            {
                for (auto& sym : headerSlice) {
                    const int shifted = loRaMod(static_cast<int>(sym) + delta, static_cast<int>(headerSymbolMod));
                    sym = static_cast<unsigned short>(shifted);
                }
            }

            bool hasCRC = true;
            unsigned int nbParityBits = 1U;
            unsigned int packetLength = 0U;
            int headerParityStatus = (int) MeshtasticDemodSettings::ParityUndefined;
            bool headerCRCStatus = false;

            MeshtasticDemodDecoderLoRa::decodeHeader(
                headerSlice,
                headerNbSymbolBits,
                hasCRC,
                nbParityBits,
                packetLength,
                headerParityStatus,
                headerCRCStatus
            );

            if (!headerCRCStatus || packetLength == 0U || nbParityBits < 1U || nbParityBits > 4U) {
                continue;
            }

            const double symbolDurationMs = (double)(1U << sf) * 1000.0 / (double)m_bandwidth;
            const bool ldro = symbolDurationMs > 16.0;
            const unsigned int sfDenom = sf - (ldro ? 2U : 0U);

            // gr-lora_sdr formula: symb_numb = 8 + ceil(max(0, 2*pay_len - sf + 2 + 5 + has_crc*4) / (sf - 2*ldro)) * (4 + cr)
            const int numerator = 2 * (int)packetLength - (int)sf + 2 + 5 + (hasCRC ? 4 : 0);
            unsigned int payloadBlocks = 0;

            if (numerator > 0 && sfDenom > 0) {
                payloadBlocks = ((unsigned int)numerator + sfDenom - 1U) / sfDenom;
            }

            const unsigned int expectedSymbols = 8U + payloadBlocks * (4U + nbParityBits);

            if (expectedSymbols > m_settings.m_nbSymbolsMax) {
                continue;
            }

            if (offset > 0U)
            {
                m_decodeMsg->dropFront(offset);
                m_loRaFrameSymbolCount = m_loRaFrameSymbolCount > offset
                    ? (m_loRaFrameSymbolCount - offset)
                    : 0U;
            }

            m_expectedSymbols = expectedSymbols;
            m_headerLocked = true;

            // qDebug("[LOOPBACK][RX] header_realign frameId=%u offset=%u delta=%d", m_loRaFrameId, offset, delta);
            qDebug("MeshtasticDemodSink::tryHeaderLock: LOCKED len=%u CR=%u CRC=%s LDRO=%s expected=%u symbols offset=%u delta=%d",
                   packetLength, nbParityBits, hasCRC ? "on" : "off", ldro ? "on" : "off", m_expectedSymbols, offset, delta);
            return;
        }
    }

    if (symbols.size() >= 10U) {
        qDebug("MeshtasticDemodSink::tryHeaderLock: header invalid after offsets 0..%u deltas -2..2", maxOffset);
    }
}

bool MeshtasticDemodSink::sendLoRaHeaderProbe()
{
    if (!m_decodeMsg || !m_decoderMsgQueue) {
        return false;
    }

    const std::vector<unsigned short>& symbols = m_decodeMsg->getSymbols();

    if (symbols.size() < 8U) {
        return false;
    }

    std::vector<unsigned short> headerSymbols(symbols.begin(), symbols.begin() + 8);
    const unsigned int payloadNbSymbolBits = (m_settings.m_spreadFactor > m_settings.m_deBits)
        ? (m_settings.m_spreadFactor - m_settings.m_deBits)
        : 1U;
    const unsigned int headerNbSymbolBits = (static_cast<unsigned int>(m_settings.m_spreadFactor) > 2U)
        ? (m_settings.m_spreadFactor - 2U)
        : payloadNbSymbolBits;

    MeshtasticDemodMsg::MsgLoRaHeaderProbe *probe = MeshtasticDemodMsg::MsgLoRaHeaderProbe::create(
        m_loRaFrameId,
        headerSymbols,
        payloadNbSymbolBits,
        headerNbSymbolBits,
        m_settings.m_spreadFactor,
        static_cast<unsigned int>(std::max(1, m_bandwidth)),
        MeshtasticDemodSettings::m_hasHeader,
        MeshtasticDemodSettings::m_hasCRC
    );
    m_decoderMsgQueue->push(probe);

    return true;
}

void MeshtasticDemodSink::applyLoRaHeaderFeedback(
    uint32_t frameId,
    bool valid,
    bool hasCRC,
    unsigned int nbParityBits,
    unsigned int packetLength,
    bool ldro,
    unsigned int expectedSymbols,
    int headerParityStatus,
    bool headerCRCStatus)
{
    (void) hasCRC;
    (void) ldro;
    (void) headerParityStatus;

    if (m_loRaState != LoRaStateSFOCompensation)
    {
        return;
    }

    if (frameId != m_loRaFrameId) {
        return;
    }

    m_waitHeaderFeedback = false;
    m_headerFeedbackWaitSteps = 0;

    if (!valid || !headerCRCStatus || (packetLength == 0U) || (nbParityBits < 1U) || (nbParityBits > 4U))
    {
        qDebug("MeshtasticDemodSink::applyLoRaHeaderFeedback: invalid header -> reset frame");
        resetLoRaFrameSync();
        return;
    }

    if (expectedSymbols > m_settings.m_nbSymbolsMax)
    {
        qDebug("MeshtasticDemodSink::applyLoRaHeaderFeedback: expected %u > max %u, fallback to EOM",
            expectedSymbols, m_settings.m_nbSymbolsMax);
        return;
    }

    m_expectedSymbols = expectedSymbols;
    m_headerLocked = true;
    m_loRaReceivedHeader = true;
}

int MeshtasticDemodSink::loRaMod(int a, int b) const
{
    if (b <= 0) {
        return 0;
    }

    return (a % b + b) % b;
}

int MeshtasticDemodSink::loRaRound(float number) const
{
    return (number > 0.0f) ? static_cast<int>(number + 0.5f) : static_cast<int>(std::ceil(number - 0.5f));
}

void MeshtasticDemodSink::resetLoRaFrameSync()
{
    if (m_decodeMsg && (m_loRaState != LoRaStateDetect))
    {
        // Preserve abandoned/invalid frames as replay evidence. Without this,
        // their capture would remain live forever and grow without bound.
        finalizeTempIqCapture(m_loRaFrameId);
        delete m_decodeMsg;
        m_decodeMsg = nullptr;
    }

    m_loRaState = LoRaStateDetect;
    m_loRaSyncState = LoRaSyncNetId1;
    m_loRaSampleFifo.clear();
    m_loRaSymbolCnt = 1;
    m_loRaBinIdx = 0;
    m_loRaKHat = 0;
    m_loRaDownVal = 0;
    m_loRaCFOInt = 0;
    m_loRaNetIdOff = 0;
    m_loRaAdditionalUpchirps = 0;
    m_loRaCFOFrac = 0.0f;
    m_loRaSTOFrac = 0.0f;
    m_loRaSFOHat = 0.0f;
    m_loRaSFOCum = 0.0f;
    m_loRaCFOSTOEstimated = false;
    m_loRaReceivedHeader = false;
    m_loRaOneSymbolOff = false;
    m_loRaFrameSymbolCount = 0;
    m_demodActive = false;
    m_headerLocked = false;
    m_expectedSymbols = 0;
    m_waitHeaderFeedback = false;
    m_headerFeedbackWaitSteps = 0;

    if (!m_loRaPreambleVals.empty()) {
        std::fill(m_loRaPreambleVals.begin(), m_loRaPreambleVals.end(), 0);
    }

    if (!m_loRaCFOFracCorrec.empty()) {
        std::fill(m_loRaCFOFracCorrec.begin(), m_loRaCFOFracCorrec.end(), Complex{1.0f, 0.0f});
    }
}

void MeshtasticDemodSink::clearSpectrumHistoryForNewFrame()
{
    if (!m_spectrumSink || !m_spectrumLine) {
        return;
    }

    // Insert a short floor separator between frames to make packet boundaries
    // visible even when consecutive packets arrive with a short idle gap.
    static constexpr unsigned int kSeparatorLines = 16U;
    for (unsigned int i = 0; i < kSeparatorLines; ++i) {
        m_spectrumSink->feed(m_spectrumLine, m_nbSymbols);
    }
}

unsigned int MeshtasticDemodSink::getLoRaSymbolVal(
    const Complex *samples,
    const Complex *refChirp,
    std::vector<float> *symbolMagnitudes,
    bool publishSpectrum
)
{
    for (unsigned int i = 0; i < m_nbSymbols; i++) {
        m_fft->in()[i] = samples[i] * refChirp[i];
    }

    // Canonical gr-lora_sdr demod uses a rectangular symbol window for
    // frame_sync/fft_demod symbol decisions. Do not apply user-selected FFT
    // windows in LoRa mode, otherwise header symbols drift and CRC checks fail.

    if (m_interpolatedFFTLength > m_fftLength) {
        std::fill(m_fft->in() + m_fftLength, m_fft->in() + m_interpolatedFFTLength, Complex{0.0f, 0.0f});
    }

    m_fft->transform();

    if (symbolMagnitudes)
    {
        symbolMagnitudes->assign(m_nbSymbols, 0.0f);

        for (unsigned int i = 0; i < m_nbSymbols; i++) {
            (*symbolMagnitudes)[i] = static_cast<float>(std::norm(m_fft->out()[i]));
        }
    }

    double magsq = 0.0;
    double magsqTotal = 0.0;
    const bool canCaptureSpectrum = (m_spectrumBuffer != nullptr);
    const bool publishSpectrumNow = publishSpectrum && (m_spectrumSink != nullptr) && canCaptureSpectrum;
    const unsigned int imax = argmax(
        m_fft->out(),
        m_fftInterpolation,
        m_fftLength,
        magsq,
        magsqTotal,
        canCaptureSpectrum ? m_spectrumBuffer : nullptr,
        m_fftInterpolation
    );

    if (publishSpectrumNow) {
        m_spectrumSink->feed(m_spectrumBuffer, m_nbSymbols);
    }

    return imax / m_fftInterpolation;
}

float MeshtasticDemodSink::estimateLoRaCFOFracBernier(const Complex *samples)
{
    if (m_loRaUpSymbToUse <= 1) {
        return 0.0f;
    }

    std::vector<int> k0(m_loRaUpSymbToUse, 0);
    std::vector<double> k0Mag(m_loRaUpSymbToUse, 0.0);
    std::vector<Complex> fftVal(m_loRaUpSymbToUse * m_nbSymbols);
    std::vector<Complex> dechirped(m_nbSymbols);

    for (int i = 0; i < m_loRaUpSymbToUse; i++)
    {
        const Complex *sym = samples + i * m_nbSymbols;

        for (unsigned int j = 0; j < m_nbSymbols; j++) {
            dechirped[j] = sym[j] * m_downChirps[j];
            m_fft->in()[j] = dechirped[j];
        }

        if (m_interpolatedFFTLength > m_fftLength) {
            std::fill(m_fft->in() + m_fftLength, m_fft->in() + m_interpolatedFFTLength, Complex{0.0f, 0.0f});
        }

        m_fft->transform();

        double magsq = 0.0;
        double magsqTotal = 0.0;
        const unsigned int imax = argmax(
            m_fft->out(),
            m_fftInterpolation,
            m_fftLength,
            magsq,
            magsqTotal,
            nullptr,
            m_fftInterpolation
        ) / m_fftInterpolation;
        k0[i] = static_cast<int>(imax);
        k0Mag[i] = magsq;

        for (unsigned int j = 0; j < m_nbSymbols; j++) {
            fftVal[j + i * m_nbSymbols] = m_fft->out()[j];
        }
    }

    const int idxMax = k0[std::distance(k0Mag.begin(), std::max_element(k0Mag.begin(), k0Mag.end()))];
    Complex fourCum(0.0f, 0.0f);

    for (int i = 0; i < m_loRaUpSymbToUse - 1; i++) {
        fourCum += fftVal[idxMax + m_nbSymbols * i] * std::conj(fftVal[idxMax + m_nbSymbols * (i + 1)]);
    }

    const float cfoFrac = -std::arg(fourCum) / (2.0f * static_cast<float>(M_PI));
    const unsigned int corrCount = static_cast<unsigned int>(m_loRaUpSymbToUse) * m_nbSymbols;

    for (unsigned int n = 0; n < corrCount && n < m_loRaPreambleUpchirps.size(); n++)
    {
        const float phase = -2.0f * static_cast<float>(M_PI) * cfoFrac * static_cast<float>(n) / static_cast<float>(m_nbSymbols);
        m_loRaPreambleUpchirps[n] = samples[n] * Complex(std::cos(phase), std::sin(phase));
    }

    return cfoFrac;
}

float MeshtasticDemodSink::estimateLoRaSTOFrac()
{
    if (m_loRaUpSymbToUse <= 0) {
        return 0.0f;
    }

    FFTFactory *fftFactory = DSPEngine::instance()->getFFTFactory();
    FFTEngine *fft2N = nullptr;
    const unsigned int fft2NLen = 2U * m_nbSymbols;
    const int fft2NSeq = fftFactory->getEngine(fft2NLen, false, &fft2N);
    std::vector<double> fftMagSq(fft2NLen, 0.0);
    std::vector<Complex> dechirped(m_nbSymbols);

    for (int i = 0; i < m_loRaUpSymbToUse; i++)
    {
        const Complex *sym = m_loRaPreambleUpchirps.data() + i * m_nbSymbols;

        for (unsigned int j = 0; j < m_nbSymbols; j++) {
            dechirped[j] = sym[j] * m_downChirps[j];
            fft2N->in()[j] = dechirped[j];
        }

        std::fill(fft2N->in() + m_nbSymbols, fft2N->in() + fft2NLen, Complex{0.0f, 0.0f});
        fft2N->transform();

        for (unsigned int j = 0; j < fft2NLen; j++) {
            fftMagSq[j] += std::norm(fft2N->out()[j]);
        }
    }

    fftFactory->releaseEngine(fft2NLen, false, fft2NSeq);

    const int k0 = static_cast<int>(std::distance(fftMagSq.begin(), std::max_element(fftMagSq.begin(), fftMagSq.end())));
    const double Y_1 = fftMagSq[loRaMod(k0 - 1, static_cast<int>(fft2NLen))];
    const double Y0 = fftMagSq[k0];
    const double Y1 = fftMagSq[loRaMod(k0 + 1, static_cast<int>(fft2NLen))];
    const double u = 64.0 * m_nbSymbols / 406.5506497;
    const double v = u * 2.4674;
    const double wa = (Y1 - Y_1) / (u * (Y1 + Y_1) + v * Y0 + 1e-12);
    const double ka = wa * m_nbSymbols / M_PI;
    const double kres = std::fmod((k0 + ka) / 2.0, 1.0);

    return static_cast<float>(kres - (kres > 0.5 ? 1.0 : 0.0));
}

void MeshtasticDemodSink::buildLoRaPayloadDownchirp()
{
    const int N = static_cast<int>(m_nbSymbols);
    const int id = loRaMod(m_loRaCFOInt, N);

    for (int n = 0; n < N; n++)
    {
        const int nFold = N - id;
        const double nD = static_cast<double>(n);
        const double ND = static_cast<double>(N);
        double phase;

        if (n < nFold) {
            phase = 2.0 * M_PI * ((nD * nD) / (2.0 * ND) + (static_cast<double>(id) / ND - 0.5) * nD);
        } else {
            phase = 2.0 * M_PI * ((nD * nD) / (2.0 * ND) + (static_cast<double>(id) / ND - 1.5) * nD);
        }

        const Complex up(std::cos(phase), std::sin(phase));
        Complex ref = m_settings.m_invertRamps ? up : std::conj(up);
        const float cfoPhase = -2.0f * static_cast<float>(M_PI) * m_loRaCFOFrac * static_cast<float>(n) / static_cast<float>(N);
        ref *= Complex(std::cos(cfoPhase), std::sin(cfoPhase));
        m_loRaPayloadDownchirp[n] = ref;
    }
}

void MeshtasticDemodSink::finalizeLoRaFrame()
{
    if (!m_decodeMsg) {
        resetLoRaFrameSync();
        return;
    }

    qDebug(
        "MeshtasticDemodSink::finalizeLoRaFrame: frameId=%u symbols=%u headerLocked=%d expected=%u",
        m_loRaFrameId,
        m_loRaFrameSymbolCount,
        m_headerLocked ? 1 : 0,
        m_expectedSymbols
    );

    m_decodeMsg->setSignalDb(CalcDb::dbPower(m_magsqOnAvg.asDouble() / (1 << m_settings.m_spreadFactor)));
    m_decodeMsg->setNoiseDb(CalcDb::dbPower(m_magsqOffAvg.asDouble() / (1 << m_settings.m_spreadFactor)));

    if (m_decoderMsgQueue && m_settings.m_decodeActive) {
        m_decoderMsgQueue->push(m_decodeMsg);
    } else {
        delete m_decodeMsg;
    }

    finalizeTempIqCapture(m_loRaFrameId);
    m_decodeMsg = nullptr;
    resetLoRaFrameSync();
}

void MeshtasticDemodSink::processSampleLoRa(const Complex& ci)
{
    m_loRaSampleFifo.push_back(ci);

    while (true)
    {
        const unsigned int needed = (m_loRaState == LoRaStateSync) ? (3U * m_loRaSymbolSpan) : m_loRaSymbolSpan;

        if (m_loRaSampleFifo.size() < needed) {
            return;
        }

        int consumed = processLoRaFrameSyncStep();

        if (consumed <= 0) {
            consumed = 1;
        }

        consumed = std::min(consumed, static_cast<int>(m_loRaSampleFifo.size()));

        for (int i = 0; i < consumed; i++) {
            m_loRaSampleFifo.pop_front();
        }
    }
}

int MeshtasticDemodSink::processLoRaFrameSyncStep()
{
    const int stoShift = loRaRound(m_loRaSTOFrac * static_cast<float>(m_osFactor));

    for (unsigned int ii = 0; ii < m_nbSymbols; ii++)
    {
        int idx = static_cast<int>(m_osCenterPhase + m_osFactor * ii) - stoShift;
        idx = std::max(0, std::min(idx, static_cast<int>(m_loRaSymbolSpan) - 1));
        m_loRaInDown[ii] = m_loRaSampleFifo[static_cast<size_t>(idx)];
    }

    if (m_loRaState == LoRaStateDetect)
    {
        const Complex *detectRef = m_settings.m_invertRamps ? m_upChirps : m_downChirps;
        // Keep last decoded packet visible until next frame starts.
        const int binNew = static_cast<int>(getLoRaSymbolVal(m_loRaInDown.data(), detectRef, nullptr, false));
        const int detectDelta = std::abs(loRaMod(std::abs(binNew - m_loRaBinIdx) + 1, static_cast<int>(m_nbSymbols)) - 1);
        const bool isConsecutive = (detectDelta <= 1);
        double symbolPower = 0.0;

        for (const Complex &s : m_loRaInDown) {
            symbolPower += std::norm(s);
        }

        symbolPower /= std::max(1U, m_nbSymbols);
        m_magsqTotalAvg(symbolPower);

        if (isConsecutive)
        {
            if (m_loRaSymbolCnt == 1 && !m_loRaPreambleVals.empty()) {
                m_loRaPreambleVals[0] = m_loRaBinIdx;
            }

            if ((m_loRaSymbolCnt >= 0) && (m_loRaSymbolCnt < static_cast<int>(m_loRaPreambleVals.size()))) {
                m_loRaPreambleVals[m_loRaSymbolCnt] = binNew;
            }

            const size_t sOfs = static_cast<size_t>(m_loRaSymbolCnt) * m_nbSymbols;
            if (sOfs + m_nbSymbols <= m_loRaPreambleRaw.size()) {
                std::copy_n(m_loRaInDown.begin(), m_nbSymbols, m_loRaPreambleRaw.begin() + sOfs);
            }

            const size_t upOfs = static_cast<size_t>(m_loRaSymbolCnt) * m_loRaSymbolSpan;
            if (upOfs + m_loRaSymbolSpan <= m_loRaPreambleRawUp.size()) {
                std::copy_n(m_loRaSampleFifo.begin(), m_loRaSymbolSpan, m_loRaPreambleRawUp.begin() + upOfs);
            }

            m_loRaSymbolCnt++;
        }
        else
        {
            m_magsqOffAvg(symbolPower);

            if (m_loRaPreambleRaw.size() >= m_nbSymbols) {
                std::copy_n(m_loRaInDown.begin(), m_nbSymbols, m_loRaPreambleRaw.begin());
            }

            if (m_loRaPreambleRawUp.size() >= m_loRaSymbolSpan) {
                std::copy_n(m_loRaSampleFifo.begin(), m_loRaSymbolSpan, m_loRaPreambleRawUp.begin());
            }

            m_loRaSymbolCnt = 1;
        }

        m_loRaBinIdx = binNew;

        if ((m_loRaSymbolCnt >= static_cast<int>(m_loRaRequiredUpchirps))
            && !m_loRaPreambleVals.empty())
        {
            m_loRaAdditionalUpchirps = 0;
            m_loRaState = LoRaStateSync;
            m_loRaSyncState = LoRaSyncNetId1;
            m_loRaSymbolCnt = 0;
            m_loRaCFOSTOEstimated = false;

            std::vector<unsigned int> hist(m_nbSymbols, 0U);
            unsigned int bestBin = 0U;
            unsigned int bestCount = 0U;

            for (int v : m_loRaPreambleVals)
            {
                const unsigned int b = static_cast<unsigned int>(loRaMod(v, static_cast<int>(m_nbSymbols)));
                const unsigned int c = ++hist[b];

                if (c > bestCount) {
                    bestCount = c;
                    bestBin = b;
                }
            }

            m_loRaKHat = static_cast<int>(bestBin);
            const int netStart = static_cast<int>(0.75f * static_cast<float>(m_loRaSymbolSpan)) - m_loRaKHat * static_cast<int>(m_osFactor);

            for (unsigned int i = 0; i < m_loRaSymbolSpan / 4U; i++)
            {
                const int src = std::max(0, std::min(netStart + static_cast<int>(i), static_cast<int>(m_loRaSampleFifo.size()) - 1));
                if (i < m_loRaNetIdSamp.size()) {
                    m_loRaNetIdSamp[i] = m_loRaSampleFifo[static_cast<size_t>(src)];
                }
            }

            return static_cast<int>(m_osFactor * (m_nbSymbols - bestBin));
        }

        return static_cast<int>(m_loRaSymbolSpan);
    }

    if (m_loRaState == LoRaStateSync)
    {
        if (!m_loRaCFOSTOEstimated)
        {
            const int cfoStart = std::max(0, static_cast<int>(m_nbSymbols) - m_loRaKHat);

            if (cfoStart < static_cast<int>(m_loRaPreambleRaw.size())) {
                m_loRaCFOFrac = estimateLoRaCFOFracBernier(m_loRaPreambleRaw.data() + cfoStart);
            } else {
                m_loRaCFOFrac = 0.0f;
            }

            m_loRaSTOFrac = estimateLoRaSTOFrac();

            for (unsigned int n = 0; n < m_nbSymbols; n++)
            {
                const float phase = -2.0f * static_cast<float>(M_PI) * m_loRaCFOFrac * static_cast<float>(n) / static_cast<float>(m_nbSymbols);
                m_loRaCFOFracCorrec[n] = Complex(std::cos(phase), std::sin(phase));
            }

            m_loRaCFOSTOEstimated = true;
        }

        for (unsigned int i = 0; i < m_nbSymbols; i++) {
            m_loRaSymbCorr[i] = m_loRaInDown[i] * m_loRaCFOFracCorrec[i];
        }

        const Complex *syncRef = m_settings.m_invertRamps ? m_upChirps : m_downChirps;
        const int binIdx = static_cast<int>(getLoRaSymbolVal(m_loRaSymbCorr.data(), syncRef, nullptr, true));

        switch (m_loRaSyncState)
        {
        case LoRaSyncNetId1:
            if ((binIdx == 0) || (binIdx == 1) || (binIdx == static_cast<int>(m_nbSymbols) - 1))
            {
                const size_t dstOfs = static_cast<size_t>(m_loRaRequiredUpchirps + m_loRaAdditionalUpchirps) * m_loRaSymbolSpan;

                if (dstOfs + m_loRaSymbolSpan <= m_loRaPreambleRawUp.size()) {
                    std::copy_n(m_loRaSampleFifo.begin(), m_loRaSymbolSpan, m_loRaPreambleRawUp.begin() + dstOfs);
                }

                m_loRaAdditionalUpchirps = std::min(m_loRaAdditionalUpchirps + 1, 3);
            }
            else
            {
                m_loRaSyncState = LoRaSyncNetId2;
                m_loRaNetIds[0] = binIdx;
            }
            break;
        case LoRaSyncNetId2:
            m_loRaSyncState = LoRaSyncDownchirp1;
            m_loRaNetIds[1] = binIdx;
            break;
        case LoRaSyncDownchirp1:
            m_loRaSyncState = LoRaSyncDownchirp2;
            break;
        case LoRaSyncDownchirp2:
            m_loRaDownVal = static_cast<int>(getLoRaSymbolVal(m_loRaSymbCorr.data(), m_settings.m_invertRamps ? m_downChirps : m_upChirps, nullptr, true));
            if (m_loRaAdditionalSymbolSamp.size() >= m_loRaSymbolSpan) {
                std::copy_n(m_loRaSampleFifo.begin(), m_loRaSymbolSpan, m_loRaAdditionalSymbolSamp.begin());
            }
            m_loRaSyncState = LoRaSyncQuarterDown;
            break;
        case LoRaSyncQuarterDown:
        default:
            if (m_loRaAdditionalSymbolSamp.size() >= 2U * m_loRaSymbolSpan) {
                std::copy_n(m_loRaSampleFifo.begin(), m_loRaSymbolSpan, m_loRaAdditionalSymbolSamp.begin() + m_loRaSymbolSpan);
            }

            if (static_cast<unsigned int>(m_loRaDownVal) < m_nbSymbols / 2U) {
                m_loRaCFOInt = static_cast<int>(std::floor(m_loRaDownVal / 2.0));
            } else {
                m_loRaCFOInt = static_cast<int>(std::floor((m_loRaDownVal - static_cast<int>(m_nbSymbols)) / 2.0));
            }

            // Preserve state-machine net ID bin indices for sync word extraction.
            // The corrLen-based refinement overwrites m_loRaNetIds but may produce
            // incorrect results when m_loRaNetIdSamp is not fully populated.
            const int netIdBin0 = m_loRaNetIds[0];
            const int netIdBin1 = m_loRaNetIds[1];

            const unsigned int upSymCount = std::min<unsigned int>(
                static_cast<unsigned int>(std::max(0, m_loRaUpSymbToUse)),
                static_cast<unsigned int>(m_loRaPreambleUpchirps.size() / std::max(1U, m_nbSymbols))
            );
            const unsigned int corrLen = upSymCount * m_nbSymbols;

            if (corrLen > 0U)
            {
                const int cfoIntMod = loRaMod(m_loRaCFOInt, static_cast<int>(m_nbSymbols));
                std::rotate(
                    m_loRaPreambleUpchirps.begin(),
                    m_loRaPreambleUpchirps.begin() + cfoIntMod,
                    m_loRaPreambleUpchirps.begin() + corrLen
                );

                std::vector<Complex> cfoIntCorrec(corrLen, Complex{1.0f, 0.0f});
                for (unsigned int n = 0; n < corrLen; n++)
                {
                    const float phase = -2.0f * static_cast<float>(M_PI)
                        * static_cast<float>(m_loRaCFOInt)
                        * static_cast<float>(n)
                        / static_cast<float>(m_nbSymbols);
                    cfoIntCorrec[n] = Complex(std::cos(phase), std::sin(phase));
                }

                for (unsigned int n = 0; n < corrLen; n++) {
                    m_loRaPreambleUpchirps[n] *= cfoIntCorrec[n];
                }

                if (m_deviceCenterFrequency > 0) {
                    m_loRaSFOHat =
                        (static_cast<float>(m_loRaCFOInt) + m_loRaCFOFrac)
                        * static_cast<float>(m_bandwidth)
                        / static_cast<float>(m_deviceCenterFrequency);
                } else {
                    m_loRaSFOHat = 0.0f;
                }

                std::vector<Complex> sfoCorrec(corrLen, Complex{1.0f, 0.0f});
                const double clkOff = static_cast<double>(m_loRaSFOHat) / static_cast<double>(m_nbSymbols);
                const double fs = static_cast<double>(m_bandwidth);
                const double fsP = fs * (1.0 - clkOff);
                const int N = static_cast<int>(m_nbSymbols);

                for (unsigned int n = 0; n < corrLen; n++)
                {
                    const double nMod = static_cast<double>(loRaMod(static_cast<int>(n), N));
                    const double nFloor = std::floor(static_cast<double>(n) / static_cast<double>(N));
                    const double q1 = (nMod * nMod) / (2.0 * static_cast<double>(N))
                        * ((m_bandwidth / fsP) * (m_bandwidth / fsP) - (m_bandwidth / fs) * (m_bandwidth / fs));
                    const double q2 = (nFloor * ((m_bandwidth / fsP) * (m_bandwidth / fsP) - (m_bandwidth / fsP))
                        + m_bandwidth / 2.0 * (1.0 / fs - 1.0 / fsP)) * nMod;
                    const double phase = -2.0 * M_PI * (q1 + q2);
                    sfoCorrec[n] = Complex(std::cos(phase), std::sin(phase));
                }

                for (unsigned int n = 0; n < corrLen; n++) {
                    m_loRaPreambleUpchirps[n] *= sfoCorrec[n];
                }

                const float tmpSto = estimateLoRaSTOFrac();
                const float diffSto = m_loRaSTOFrac - tmpSto;

                if (std::abs(diffSto) <= (static_cast<float>(m_osFactor) - 1.0f) / static_cast<float>(m_osFactor)) {
                    m_loRaSTOFrac = tmpSto;
                }

                std::vector<Complex> netIdsDec(2U * m_nbSymbols, Complex{0.0f, 0.0f});
                const int startOff = static_cast<int>(m_osFactor / 2U)
                    - loRaRound(m_loRaSTOFrac * static_cast<float>(m_osFactor))
                    + static_cast<int>(m_osFactor)
                        * (static_cast<int>(0.25f * static_cast<float>(m_nbSymbols)) + m_loRaCFOInt);

                for (unsigned int i = 0; i < 2U * m_nbSymbols; i++)
                {
                    const int idx = std::max(
                        0,
                        std::min(startOff + static_cast<int>(i * m_osFactor), static_cast<int>(m_loRaNetIdSamp.size()) - 1)
                    );
                    netIdsDec[i] = m_loRaNetIdSamp[static_cast<size_t>(idx)];
                }

                for (unsigned int i = 0; i < 2U * m_nbSymbols; i++)
                {
                    netIdsDec[i] *= cfoIntCorrec[i % std::max(1U, corrLen)];

                    const float phase = -2.0f * static_cast<float>(M_PI)
                        * m_loRaCFOFrac
                        * static_cast<float>(i % m_nbSymbols)
                        / static_cast<float>(m_nbSymbols);
                    netIdsDec[i] *= Complex(std::cos(phase), std::sin(phase));
                }

                const int netid1 = static_cast<int>(getLoRaSymbolVal(netIdsDec.data(), m_settings.m_invertRamps ? m_upChirps : m_downChirps));
                const int netid2 = static_cast<int>(getLoRaSymbolVal(netIdsDec.data() + m_nbSymbols, m_settings.m_invertRamps ? m_upChirps : m_downChirps));
                m_loRaNetIds[0] = netid1;
                m_loRaNetIds[1] = netid2;
                m_loRaNetIdOff = netid1;
            }
            else
            {
                if (m_deviceCenterFrequency > 0) {
                    m_loRaSFOHat =
                        (static_cast<float>(m_loRaCFOInt) + m_loRaCFOFrac)
                        * static_cast<float>(m_bandwidth)
                        / static_cast<float>(m_deviceCenterFrequency);
                } else {
                    m_loRaSFOHat = 0.0f;
                }
            }

            buildLoRaPayloadDownchirp();
            m_loRaFrameId++;
            m_decodeMsg = MeshtasticDemodMsg::MsgDecodeSymbols::create();
            m_decodeMsg->setFrameId(m_loRaFrameId);
            TempIqCapture *tempIqCapture = startTempIqCapture(m_loRaFrameId);
            if (tempIqCapture)
            {
                m_decodeMsg->setTempIqCapture(
                    tempIqCapture->filePath,
                    tempIqCapture->sampleRate,
                    tempIqCapture->preRollSamples,
                    tempIqCapture->postRollSamples
                );
            }
            // Freeze the RF and pipeline configuration that produced this frame.
            // Later channel or pipeline reconfiguration must not change the
            // provenance reported for this already-created frame.
            m_decodeMsg->setRFMetadata(
                m_deviceCenterFrequency + m_settings.m_inputFrequencyOffset,
                static_cast<unsigned int>(m_bandwidth),
                static_cast<unsigned int>(m_settings.m_spreadFactor),
                m_settings.m_meshtasticPresetName
            );
            // Capture the raw RF state when the frame is captured for later
            // comparison with the channel state present when JSON is generated.
            m_decodeMsg->setRFDiagnostics(
                m_deviceCenterFrequency,
                m_settings.m_inputFrequencyOffset,
                m_channelFrequencyOffset
            );
            {
                // LoRa sync word is encoded across two net-ID chirps.
                // First chirp (index 0) carries the high nibble, second (index 1) the low nibble.
                // Each nibble N is mapped to bin N*8. Formula: syncWord = low + 16*high.
                // Use the coarse state-machine bin values (netIdBin0/1) which are reliably
                // set from the FFT in LoRaSyncNetId1/2 states, not the refined values which
                // require m_loRaNetIdSamp to be fully populated.
                const unsigned int hiNibble = static_cast<unsigned int>(std::round(static_cast<double>(netIdBin0) / 8.0)) & 0xFU;
                const unsigned int loNibble = static_cast<unsigned int>(std::round(static_cast<double>(netIdBin1) / 8.0)) & 0xFU;
                m_decodeMsg->setSyncWord(loNibble + 16U * hiNibble);
            }
            clearSpectrumHistoryForNewFrame();
            m_loRaFrameSymbolCount = 0U;
            m_magsqMax = 0.0;
            m_headerLocked = false;
            m_expectedSymbols = 0;
            m_waitHeaderFeedback = false;
            m_headerFeedbackWaitSteps = 0U;
            m_demodActive = true;
            m_loRaSTOFrac += m_loRaSFOHat * 4.25f;
            if (std::abs(m_loRaSTOFrac) > 0.5f) {
                m_loRaSTOFrac += (m_loRaSTOFrac > 0.0f) ? -1.0f : 1.0f;
            }
            const float stoQuant = static_cast<float>(loRaRound(m_loRaSTOFrac * static_cast<float>(m_osFactor)));
            m_loRaSFOCum = ((m_loRaSTOFrac * static_cast<float>(m_osFactor)) - stoQuant) / static_cast<float>(m_osFactor);
            m_loRaState = LoRaStateSFOCompensation;
            m_loRaSyncState = LoRaSyncNetId1;
            return std::max(1, static_cast<int>(m_loRaSymbolSpan / 4U + static_cast<int>(m_osFactor) * m_loRaCFOInt));
        }

        return static_cast<int>(m_loRaSymbolSpan);
    }

    if (!m_decodeMsg)
    {
        resetLoRaFrameSync();
        return static_cast<int>(m_loRaSymbolSpan);
    }

    std::vector<float> symbolMags;
    const unsigned int rawSymbol = getLoRaSymbolVal(m_loRaInDown.data(), m_loRaPayloadDownchirp.data(), &symbolMags, true);
    const bool headerSymbol = m_loRaFrameSymbolCount < 8U;
    const unsigned short symbol = evalSymbol(rawSymbol, headerSymbol) % m_nbSymbolsEff;
    m_decodeMsg->pushBackSymbol(symbol);
    m_decodeMsg->pushBackMagnitudes(symbolMags);

    if (m_spectrumBuffer)
    {
        std::vector<float> spectrumLine;
        spectrumLine.reserve(m_nbSymbols);

        for (unsigned int i = 0; i < m_nbSymbols; ++i) {
            spectrumLine.push_back(static_cast<float>(std::norm(m_spectrumBuffer[i])));
        }

        m_decodeMsg->pushBackDechirpedSpectrumLine(spectrumLine);
    }

    double magsq = 0.0;
    for (const Complex &s : m_loRaInDown) {
        magsq += std::norm(s);
    }
    magsq /= std::max(1U, m_nbSymbols);

    if (magsq > m_magsqMax) {
        m_magsqMax = magsq;
    }

    m_magsqTotalAvg(magsq);
    m_magsqOnAvg(magsq);
    m_loRaFrameSymbolCount++;

    if (!m_headerLocked
        && (m_loRaFrameSymbolCount >= 8U))
    {
        tryHeaderLock();
    }

    if (m_headerLocked)
    {
        if (m_loRaFrameSymbolCount >= m_expectedSymbols) {
            finalizeLoRaFrame();
        }
    }
    else if (m_loRaFrameSymbolCount >= m_settings.m_nbSymbolsMax)
    {
        finalizeLoRaFrame();
    }

    int itemsToConsume = static_cast<int>(m_loRaSymbolSpan);

    if (std::abs(m_loRaSFOCum) > (1.0f / (2.0f * static_cast<float>(m_osFactor))))
    {
        const int step = std::signbit(m_loRaSFOCum) ? -1 : 1;
        itemsToConsume -= step;
        m_loRaSFOCum -= step * (1.0f / static_cast<float>(m_osFactor));
    }

    m_loRaSFOCum += m_loRaSFOHat;
    return std::max(1, itemsToConsume);
}

void MeshtasticDemodSink::updateTempIqCaptureGeometry()
{
    constexpr size_t captureCeilingBytes =
        static_cast<size_t>(256U) * static_cast<size_t>(1024U) * static_cast<size_t>(1024U);
    constexpr size_t bytesPerIqSample = static_cast<size_t>(2U) * sizeof(float);
    constexpr size_t captureCeilingSamples = captureCeilingBytes / bytesPerIqSample;

    const qint64 deviceCenterFrequency = m_deviceCenterFrequency;
    const int inputFrequencyOffset = m_settings.m_inputFrequencyOffset;
    const int channelFrequencyOffset = m_channelFrequencyOffset;
    const unsigned int channelRate = static_cast<unsigned int>(std::max(1, m_channelSampleRate));
    const unsigned int bandwidth = static_cast<unsigned int>(std::max(1, m_bandwidth));
    const unsigned int nbSymbols = std::max(1U, m_nbSymbols);

    bool enabled = false;
    size_t channelSamplesPerSymbol = 0U;
    size_t preRollLimit = 0U;
    size_t maxCaptureSamples = 0U;
    size_t symbolTerm = 0U;

    const double channelSamplesPerSymbolDouble = std::round(
        static_cast<double>(channelRate)
        * static_cast<double>(nbSymbols)
        / static_cast<double>(bandwidth)
    );
    const bool samplesPerSymbolFits =
        std::isfinite(channelSamplesPerSymbolDouble)
        && (channelSamplesPerSymbolDouble >= 1.0)
        && (channelSamplesPerSymbolDouble <= static_cast<double>(captureCeilingSamples));

    if (samplesPerSymbolFits) {
        channelSamplesPerSymbol = static_cast<size_t>(channelSamplesPerSymbolDouble);
    }

    const size_t extraSymbols =
        static_cast<size_t>(m_tempIqPostRollSymbols)
        + static_cast<size_t>(m_tempIqCaptureMarginSymbols);
    const size_t nbSymbolsMax = static_cast<size_t>(m_settings.m_nbSymbolsMax);
    const bool symbolTermFits =
        nbSymbolsMax <= (std::numeric_limits<size_t>::max() - extraSymbols);

    if (symbolTermFits) {
        symbolTerm = nbSymbolsMax + extraSymbols;
    }

    const bool preRollFits =
        (channelSamplesPerSymbol != 0U)
        && (static_cast<size_t>(m_tempIqPreRollSymbols)
            <= (captureCeilingSamples / channelSamplesPerSymbol));

    if (preRollFits) {
        preRollLimit =
            static_cast<size_t>(m_tempIqPreRollSymbols) * channelSamplesPerSymbol;
    }

    if (samplesPerSymbolFits
        && symbolTermFits
        && preRollFits
        && (symbolTerm
            <= ((captureCeilingSamples - preRollLimit) / channelSamplesPerSymbol)))
    {
        enabled = true;
        maxCaptureSamples =
            preRollLimit + symbolTerm * channelSamplesPerSymbol;
    }

    // Diagnostic only. Keep this in floating point so an oversized request can
    // still be reported even when its true byte count is not representable by size_t.
    const double requestedBytes =
        (
            static_cast<double>(m_tempIqPreRollSymbols)
            + static_cast<double>(m_settings.m_nbSymbolsMax)
            + static_cast<double>(m_tempIqPostRollSymbols)
            + static_cast<double>(m_tempIqCaptureMarginSymbols)
        )
        * channelSamplesPerSymbolDouble
        * static_cast<double>(bytesPerIqSample);

    const bool captureConfigurationChanged =
        (deviceCenterFrequency != m_tempIqDeviceCenterFrequency)
        || (inputFrequencyOffset != m_tempIqInputFrequencyOffset)
        || (channelFrequencyOffset != m_tempIqChannelFrequencyOffset)
        || (channelRate != m_tempIqSampleRate)
        || (bandwidth != static_cast<unsigned int>(std::max(0, m_tempIqBandwidth)))
        || (nbSymbols != m_tempIqNbSymbols)
        || (channelSamplesPerSymbol != m_tempIqChannelSamplesPerSymbol)
        || (preRollLimit != m_tempIqPreRollLimit)
        || (maxCaptureSamples != m_tempIqMaxCaptureSamples)
        || (enabled != m_tempIqCaptureEnabled);

    if (!captureConfigurationChanged) {
        return;
    }

    // A capture must contain samples from exactly one configuration. Abort any
    // active capture before installing the new signature, and discard pre-roll
    // collected under the previous signature.
    for (TempIqCapture& capture : m_tempIqCaptures)
    {
        capture.truncated = true;
        capture.frameFinalized = true;
        capture.postRemaining = 0U;
        writeTempIqCapture(std::move(capture));
    }
    m_tempIqCaptures.clear();
    m_tempIqPreRoll.clear();

    qInfo().noquote()
        << QStringLiteral(
            "MESHTASTIC_IQ_GEOMETRY frame=%1 "
            "old_center=%2 new_center=%3 "
            "old_requested_offset=%4 new_requested_offset=%5 "
            "old_residual_offset=%6 new_residual_offset=%7 "
            "old_sample_rate=%8 new_sample_rate=%9 "
            "old_bandwidth=%10 new_bandwidth=%11 "
            "old_nb_symbols=%12 new_nb_symbols=%13 "
            "old_samples_per_symbol=%14 new_samples_per_symbol=%15 "
            "old_pre_roll_max=%16 new_pre_roll_max=%17 "
            "old_max=%18 new_max=%19 "
            "old_enabled=%20 new_enabled=%21 "
            "requested_bytes=%22 ceiling_bytes=%23")
            .arg(m_loRaFrameId)
            .arg(m_tempIqDeviceCenterFrequency)
            .arg(deviceCenterFrequency)
            .arg(m_tempIqInputFrequencyOffset)
            .arg(inputFrequencyOffset)
            .arg(m_tempIqChannelFrequencyOffset)
            .arg(channelFrequencyOffset)
            .arg(m_tempIqSampleRate)
            .arg(channelRate)
            .arg(m_tempIqBandwidth)
            .arg(bandwidth)
            .arg(m_tempIqNbSymbols)
            .arg(nbSymbols)
            .arg(m_tempIqChannelSamplesPerSymbol)
            .arg(channelSamplesPerSymbol)
            .arg(m_tempIqPreRollLimit)
            .arg(preRollLimit)
            .arg(m_tempIqMaxCaptureSamples)
            .arg(maxCaptureSamples)
            .arg(m_tempIqCaptureEnabled ? 1 : 0)
            .arg(enabled ? 1 : 0)
            .arg(requestedBytes, 0, 'g', 17)
            .arg(captureCeilingBytes);

    m_tempIqDeviceCenterFrequency = deviceCenterFrequency;
    m_tempIqInputFrequencyOffset = inputFrequencyOffset;
    m_tempIqChannelFrequencyOffset = channelFrequencyOffset;
    m_tempIqBandwidth = static_cast<int>(bandwidth);
    m_tempIqNbSymbols = nbSymbols;
    m_tempIqCaptureEnabled = enabled;
    m_tempIqSampleRate = channelRate;
    m_tempIqChannelSamplesPerSymbol = channelSamplesPerSymbol;
    m_tempIqPreRollLimit = preRollLimit;
    m_tempIqMaxCaptureSamples = maxCaptureSamples;
}

void MeshtasticDemodSink::updateTempIqCapture(const Complex& ci)
{
    if (!m_tempIqCaptureEnabled) {
        return;
    }

    m_tempIqPreRoll.push_back(ci);

    while (m_tempIqPreRoll.size() > m_tempIqPreRollLimit) {
        m_tempIqPreRoll.pop_front();
    }

    for (TempIqCapture& capture : m_tempIqCaptures)
    {
        capture.samples.push_back(ci);

        if (capture.frameFinalized && (capture.postRemaining > 0U)) {
            --capture.postRemaining;
        }

        // Safety cap for any frame-abandon path not handled elsewhere.
        // The bound is frozen when the capture starts so later geometry changes
        // cannot silently change this capture's LIMIT semantics.
        if (!capture.frameFinalized
            && (capture.samples.size() >= capture.maxSamples))
        {
            qWarning().noquote()
                << QStringLiteral("MESHTASTIC_IQ_CAPTURE_LIMIT frame=%1 samples=%2 max=%3")
                    .arg(capture.frameId)
                    .arg(capture.samples.size())
                    .arg(capture.maxSamples);
            capture.truncated = true;
            capture.frameFinalized = true;
            capture.postRemaining = 0U;
        }
    }

    for (auto it = m_tempIqCaptures.begin(); it != m_tempIqCaptures.end(); )
    {
        if (it->frameFinalized && (it->postRemaining == 0U))
        {
            writeTempIqCapture(std::move(*it));
            it = m_tempIqCaptures.erase(it);
        }
        else {
            ++it;
        }
    }
}

MeshtasticDemodSink::TempIqCapture *MeshtasticDemodSink::startTempIqCapture(uint32_t frameId)
{
    if (!m_tempIqCaptureEnabled || (m_tempIqMaxCaptureSamples == 0U)) {
        return nullptr;
    }

    TempIqCapture capture;
    capture.frameId = frameId;
    capture.preRollSamples = static_cast<unsigned int>(m_tempIqPreRoll.size());
    capture.postRollSamples = static_cast<unsigned int>(
        static_cast<size_t>(m_tempIqPostRollSymbols) * m_tempIqChannelSamplesPerSymbol);
    capture.postRemaining = capture.postRollSamples;
    capture.sampleRate = m_tempIqSampleRate;
    capture.maxSamples = m_tempIqMaxCaptureSamples;
    capture.samples.reserve(capture.maxSamples);
    capture.samples.insert(
        capture.samples.end(),
        m_tempIqPreRoll.begin(),
        m_tempIqPreRoll.end()
    );

    const QString captureDir = QStandardPaths::writableLocation(QStandardPaths::DocumentsLocation)
        + QStringLiteral("/SDRangel/meshtastic-iq-captures");
    QDir().mkpath(captureDir);
    const QString timestamp = QDateTime::currentDateTimeUtc().toString(QStringLiteral("yyyyMMdd-HHmmss-zzz"));
    const QString uniqueSuffix = QUuid::createUuid().toString(QUuid::WithoutBraces);
    capture.filePath = QStringLiteral("%1/frame-%2-%3-%4.cf32")
        .arg(captureDir)
        .arg(frameId)
        .arg(timestamp)
        .arg(uniqueSuffix);

    m_tempIqCaptures.push_back(std::move(capture));
    return &m_tempIqCaptures.back();
}

void MeshtasticDemodSink::finalizeTempIqCapture(uint32_t frameId)
{
    for (TempIqCapture& capture : m_tempIqCaptures)
    {
        if (capture.frameId == frameId)
        {
            capture.frameFinalized = true;

            if (capture.postRollSamples == 0U) {
                capture.postRemaining = 0U;
            }

            return;
        }
    }
}

void MeshtasticDemodSink::writeTempIqCapture(TempIqCapture capture)
{
    // File conversion and I/O are deliberately moved off the DSP thread.
    // The completed capture owns its sample vector, so the worker does not
    // reference sink state after handoff.
    std::thread([capture = std::move(capture)]() mutable {
        const QString finalPath = capture.filePath;
        const QString partPath = finalPath + QStringLiteral(".part");
        constexpr size_t bytesPerSample = 2U * sizeof(float);
        const size_t maxByteArrayBytes = static_cast<size_t>(std::numeric_limits<qsizetype>::max());

        if ((capture.samples.size() > (maxByteArrayBytes / bytesPerSample)))
        {
            qWarning().noquote()
                << QStringLiteral("MESHTASTIC_IQ_CAPTURE_WRITE_FAIL frame=%1 reason=size_overflow samples=%2 file=%3")
                    .arg(capture.frameId)
                    .arg(capture.samples.size())
                    .arg(partPath);
            return;
        }

        if (QFile::exists(finalPath) || QFile::exists(partPath))
        {
            qWarning().noquote()
                << QStringLiteral("MESHTASTIC_IQ_CAPTURE_WRITE_FAIL frame=%1 reason=path_exists file=%2")
                    .arg(capture.frameId)
                    .arg(QFile::exists(finalPath) ? finalPath : partPath);
            return;
        }

        const qsizetype expectedBytes =
            static_cast<qsizetype>(capture.samples.size() * bytesPerSample);

        QFile file(partPath);

        if (!file.open(QIODevice::WriteOnly))
        {
            qWarning().noquote()
                << QStringLiteral("MESHTASTIC_IQ_CAPTURE_WRITE_FAIL frame=%1 reason=open file=%2 error=%3")
                    .arg(capture.frameId)
                    .arg(partPath)
                    .arg(file.errorString());
            return;
        }

        QByteArray raw;
        raw.resize(expectedBytes);
        char *dst = raw.data();

        for (const Complex& sample : capture.samples)
        {
            const float re = sample.real();
            const float im = sample.imag();
            quint32 reBits = 0U;
            quint32 imBits = 0U;
            std::memcpy(&reBits, &re, sizeof(float));
            std::memcpy(&imBits, &im, sizeof(float));
            reBits = qToLittleEndian(reBits);
            imBits = qToLittleEndian(imBits);
            std::memcpy(dst, &reBits, sizeof(quint32));
            dst += sizeof(quint32);
            std::memcpy(dst, &imBits, sizeof(quint32));
            dst += sizeof(quint32);
        }

        const qint64 written = file.write(raw);
        const bool flushOk = file.flush();
        const QString writeError = file.errorString();
        file.close();

        const bool fullWrite =
            (written == static_cast<qint64>(expectedBytes)) && flushOk;

        if (!fullWrite)
        {
            qWarning().noquote()
                << QStringLiteral("MESHTASTIC_IQ_CAPTURE_WRITE_FAIL frame=%1 reason=incomplete expected=%2 written=%3 flush=%4 file=%5 error=%6")
                    .arg(capture.frameId)
                    .arg(expectedBytes)
                    .arg(written)
                    .arg(flushOk ? 1 : 0)
                    .arg(partPath)
                    .arg(writeError);
            return;
        }

        if (capture.truncated)
        {
            qWarning().noquote()
                << QStringLiteral("MESHTASTIC_IQ_CAPTURE_INCOMPLETE frame=%1 reason=truncated samples=%2 sample_rate=%3 pre=%4 post=%5 bytes=%6 file=%7")
                    .arg(capture.frameId)
                    .arg(capture.samples.size())
                    .arg(capture.sampleRate)
                    .arg(capture.preRollSamples)
                    .arg(capture.postRollSamples)
                    .arg(written)
                    .arg(partPath);
            return;
        }

        if (QFile::exists(finalPath))
        {
            qWarning().noquote()
                << QStringLiteral("MESHTASTIC_IQ_CAPTURE_RENAME_FAIL frame=%1 reason=destination_exists part=%2 final=%3")
                    .arg(capture.frameId)
                    .arg(partPath)
                    .arg(finalPath);
            return;
        }

        if (!QFile::rename(partPath, finalPath))
        {
            qWarning().noquote()
                << QStringLiteral("MESHTASTIC_IQ_CAPTURE_RENAME_FAIL frame=%1 part=%2 final=%3")
                    .arg(capture.frameId)
                    .arg(partPath)
                    .arg(finalPath);
            return;
        }

        qInfo().noquote()
            << QStringLiteral("MESHTASTIC_IQ_CAPTURE frame=%1 samples=%2 sample_rate=%3 pre=%4 post=%5 bytes=%6 file=%7")
                .arg(capture.frameId)
                .arg(capture.samples.size())
                .arg(capture.sampleRate)
                .arg(capture.preRollSamples)
                .arg(capture.postRollSamples)
                .arg(written)
                .arg(finalPath);
    }).detach();
}

void MeshtasticDemodSink::applyChannelSettings(int channelSampleRate, int bandwidth, int channelFrequencyOffset, bool force)
{
    qDebug() << "MeshtasticDemodSink::applyChannelSettings:"
            << " channelSampleRate: " << channelSampleRate
            << " channelFrequencyOffset: " << channelFrequencyOffset
            << " bandwidth: " << bandwidth;

    if ((channelFrequencyOffset != m_channelFrequencyOffset) ||
        (channelSampleRate != m_channelSampleRate) || force)
    {
        m_nco.setFreq(-channelFrequencyOffset, channelSampleRate);
    }

    if ((channelSampleRate != m_channelSampleRate) ||
        (bandwidth != m_bandwidth) || force)
    {
        const int targetFrameSyncRate = std::max(1, bandwidth * static_cast<int>(m_osFactor));
        // Keep the anti-alias/channel filter narrow around the configured LoRa bandwidth.
        // A too-wide cutoff destabilizes preamble bin tracking in DETECT.
        const int previousBandwidth = m_bandwidth;
        const double oldCutoffHz = static_cast<double>(previousBandwidth) / 1.9;
        const double newCutoffHz = static_cast<double>(bandwidth) / 1.9;
        qInfo().noquote()
            << QStringLiteral("MESHTASTIC_INTERP_CREATE ts=%1 frame=%2 reason=apply channel_sr=%3 requested_bw=%4 previous_bw=%5 force=%6 old_cutoff_hz=%7 new_cutoff_hz=%8")
                .arg(QDateTime::currentDateTime().toString(Qt::ISODateWithMs))
                .arg(m_loRaFrameId)
                .arg(channelSampleRate)
                .arg(bandwidth)
                .arg(previousBandwidth)
                .arg(force ? 1 : 0)
                .arg(oldCutoffHz, 0, 'f', 3)
                .arg(newCutoffHz, 0, 'f', 3);
        // Preserve the original float cutoff expression exactly; old/new
        // cutoff values above are diagnostics only.
        m_interpolator.create(16, channelSampleRate, bandwidth / 1.9f);
        m_interpolatorDistance = (Real) channelSampleRate / (Real) targetFrameSyncRate;
        m_sampleDistanceRemain = 0;
        m_osCounter = 0;
        qDebug() << "MeshtasticDemodSink::applyChannelSettings: m_interpolator.create:"
            << " m_interpolatorDistance: " << m_interpolatorDistance
            << " targetFrameSyncRate: " << targetFrameSyncRate
            << " osFactor: " << m_osFactor;
    }

    m_channelSampleRate = channelSampleRate;
    m_bandwidth = bandwidth;
    m_channelFrequencyOffset = channelFrequencyOffset;
    updateTempIqCaptureGeometry();
}

void MeshtasticDemodSink::applySettings(const MeshtasticDemodSettings& settings, bool force)
{
    qDebug() << "MeshtasticDemodSink::applySettings:"
            << " m_inputFrequencyOffset: " << settings.m_inputFrequencyOffset
            << " m_bandwidthIndex: " << settings.m_bandwidthIndex
            << " m_spreadFactor: " << settings.m_spreadFactor
            << " m_rgbColor: " << settings.m_rgbColor
            << " m_title: " << settings.m_title
            << " force: " << force;

    const unsigned int desiredFFTInterpolation = m_loRaFFTInterpolation;
    const bool fftInterpChanged = desiredFFTInterpolation != m_fftInterpolation;

    if ((settings.m_spreadFactor != m_settings.m_spreadFactor)
     || (settings.m_deBits != m_settings.m_deBits)
     || fftInterpChanged
     || force)
    {
        m_fftInterpolation = desiredFFTInterpolation;
        initSF(settings.m_spreadFactor, settings.m_deBits);
    }

    const unsigned int configuredPreamble = settings.m_preambleChirps > 0U
        ? settings.m_preambleChirps
        : m_minRequiredPreambleChirps;
    const unsigned int targetRequired = configuredPreamble > 3U
        ? (configuredPreamble - 3U)
        : m_minRequiredPreambleChirps;
    m_requiredPreambleChirps = std::max(
        m_minRequiredPreambleChirps,
        std::min(targetRequired, m_maxRequiredPreambleChirps)
    );
    qDebug() << "MeshtasticDemodSink::applySettings:"
            << " requiredPreambleChirps: " << m_requiredPreambleChirps
            << " configuredPreamble: " << settings.m_preambleChirps
            << " fftInterpolation: " << m_fftInterpolation;

    m_settings = settings;
    updateTempIqCaptureGeometry();
    m_loRaRequiredUpchirps = m_requiredPreambleChirps;
    m_loRaUpSymbToUse = (m_loRaRequiredUpchirps > 0U) ? static_cast<int>(m_loRaRequiredUpchirps - 1U) : 0;
    m_loRaPreambleVals.assign(m_loRaRequiredUpchirps, 0);
    m_loRaPreambleRaw.assign(m_nbSymbols * m_loRaRequiredUpchirps, Complex{0.0f, 0.0f});
    m_loRaPreambleUpchirps.assign(m_nbSymbols * m_loRaRequiredUpchirps, Complex{0.0f, 0.0f});
    m_loRaPreambleRawUp.assign((m_settings.m_preambleChirps + 3U) * m_loRaSymbolSpan, Complex{0.0f, 0.0f});
    m_loRaCFOFracCorrec.assign(m_nbSymbols, Complex{1.0f, 0.0f});
    m_loRaPayloadDownchirp.assign(m_nbSymbols, Complex{1.0f, 0.0f});
    m_loRaSymbCorr.assign(m_nbSymbols, Complex{0.0f, 0.0f});
    m_loRaNetIdSamp.assign((m_loRaSymbolSpan * 5U) / 2U + m_loRaSymbolSpan, Complex{0.0f, 0.0f});
    m_loRaAdditionalSymbolSamp.assign(m_loRaSymbolSpan * 2U, Complex{0.0f, 0.0f});
    resetLoRaFrameSync();
}
