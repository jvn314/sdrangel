///////////////////////////////////////////////////////////////////////////////////
// This program is free software; you can redistribute it and/or modify          //
// it under the terms of the GNU General Public License as published by          //
// the Free Software Foundation as version 3 of the License, or                  //
// (at your option) any later version.                                           //
//                                                                               //
// This program is distributed in the hope that it will be useful,               //
// but WITHOUT ANY WARRANTY; without even the implied warranty of                //
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the                 //
// GNU General Public License V3 for more details.                               //
//                                                                               //
// You should have received a copy of the GNU General Public License             //
// along with this program. If not, see <http://www.gnu.org/licenses/>.          //
///////////////////////////////////////////////////////////////////////////////////

#ifndef INCLUDE_MESHTASTICDEMODFILELOG_H
#define INCLUDE_MESHTASTICDEMODFILELOG_H

#include <QByteArray>
#include <QDateTime>
#include <QDir>
#include <QFile>
#include <QJsonDocument>
#include <QJsonObject>
#include <QMutex>
#include <QMutexLocker>
#include <QString>

// Diagnostic file log for the Meshtastic demodulator.
// One compact JSON object per line, appended to <user home>/angel_log_1.
// On Windows QDir::homePath() is the user profile directory (%USERPROFILE%),
// so the file is %USERPROFILE%\angel_log_1.
// Safe to call from the sink (DSP thread) and the decoder thread.
namespace MeshtasticDemodFileLog
{
    inline QString path()
    {
        return QDir::toNativeSeparators(QDir(QDir::homePath()).filePath(QStringLiteral("angel_log_1")));
    }

    inline void append(QJsonObject record)
    {
        static QMutex mutex;
        record.insert(QStringLiteral("ts"), QDateTime::currentDateTimeUtc().toString(Qt::ISODateWithMs));
        QByteArray line = QJsonDocument(record).toJson(QJsonDocument::Compact);
        line.append('\n');

        QMutexLocker locker(&mutex);
        QFile file(path());

        if (file.open(QIODevice::WriteOnly | QIODevice::Append)) {
            file.write(line);
        }
    }
}

#endif // INCLUDE_MESHTASTICDEMODFILELOG_H
