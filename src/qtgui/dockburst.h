/* -*- c++ -*- */
/*
 * Gqrx SDR: Software defined radio receiver powered by GNU Radio and Qt
 *           http://gqrx.dk/
 *
 * Copyright 2011-2016 Alexandru Csete OZ9AEC.
 *
 * Gqrx is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 3, or (at your option)
 * any later version.
 *
 * Gqrx is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Gqrx; see the file COPYING.  If not, write to
 * the Free Software Foundation, Inc., 51 Franklin Street,
 * Boston, MA 02110-1301, USA.
 */
#ifndef DOCKBURST_H
#define DOCKBURST_H

#include <QColor>
#include <QDockWidget>
#include <QSettings>
#include "audio_options.h"

namespace Ui {
    class DockBurst;
}


/*! \brief Dock window with audio options.
 *  \ingroup UI
 *
 * This dock widget encapsulates the audio options.
 * The UI itself is in the dockaudio.ui file.
 *
 * This class also provides the signal/slot API necessary to connect
 * the encapsulated widgets to the rest of the application.
 */
class DockBurst : public QDockWidget
{
    Q_OBJECT

public:
    explicit DockBurst(QWidget *parent = 0);
    ~DockBurst();

    void setFftRange(quint64 minf, quint64 maxf);
    void setSampleRate(float Rate);
    void ResetPeakHold();
    void setNewFttData(float *fftData, int size);
    int  fftRate() const { return 10; }

    void setBurstRecButtonState(bool checked);

    void setFftColor(QColor color);
    void setFftFill(bool enabled);

    void saveSettings(QSettings *settings);
    void readSettings(QSettings *settings);

public slots:
    void startBurstRecorder(void);
    void stopBurstRecorder(void);
    void setRxFrequency(qint64 freq);

signals:

    /*! \brief Audio streaming over UDP has started. */
    void audioStreamingStarted(const QString host, int port);

    /*! \brief Audio streaming stopped. */
    void audioStreamingStopped();

    /*! \brief Signal emitted when audio recording is started. */
    void burstRecStarted(const QString filename);

    /*! \brief Signal emitted when audio recording is stopped. */
    void burstRecStopped();


    /*! \brief FFT rate changed. */
    void fftRateChanged(int fps);

private slots:
    void on_audioStreamButton_clicked(bool checked);
    void on_burstRecButton_clicked(bool checked);
    void on_audioConfButton_clicked();
    void setNewPandapterRange(int min, int max);
    void setNewWaterfallRange(int min, int max);
    void setNewRecDir(const QString &dir);
    void setNewUdpHost(const QString &host);
    void setNewUdpPort(int port);


private:
    Ui::DockBurst  *ui;
    CAudioOptions *audioOptions; /*! Audio options dialog. */
    QString        rec_dir;      /*! Location for audio recordings. */
    QString        last_burst;   /*! Last audio recording. */

    QString        udp_host;     /*! UDP client host name. */
    int            udp_port;     /*! UDP client port number. */

    bool           autoSpan;     /*! Whether to allow mode-dependent auto span. */

    qint64         rx_freq;      /*! RX frequency used in filenames. */
};

#endif // DOCKBURST_H
