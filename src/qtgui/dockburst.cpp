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
#include <QDebug>
#include <QDateTime>
#include <QDir>
#include "dockburst.h"
#include "ui_dockburst.h"

DockBurst::DockBurst(QWidget *parent) :
    QDockWidget(parent),
    ui(new Ui::DockBurst),
    autoSpan(true),
    rx_freq(144000000)
{
    ui->setupUi(this);

#ifdef Q_OS_MAC
    // Workaround for Mac, see http://stackoverflow.com/questions/3978889/why-is-qhboxlayout-causing-widgets-to-overlap
    // Might be fixed in Qt 5?
    ui->burstRecButton->setAttribute(Qt::WA_LayoutUsesWidgetRect);
    ui->burstConfButton->setAttribute(Qt::WA_LayoutUsesWidgetRect);
#endif

#ifdef Q_OS_LINUX
    // buttons can be smaller than 50x32
    ui->audioStreamButton->setMinimumSize(48, 24);
    ui->burstRecButton->setMinimumSize(48, 24);;
    ui->audioConfButton->setMinimumSize(48, 24);
#endif

    audioOptions = new CAudioOptions(this);

    connect(audioOptions, SIGNAL(newFftSplit(int)), ui->burstSpectrum, SLOT(setPercent2DScreen(int)));
    connect(audioOptions, SIGNAL(newPandapterRange(int,int)), this, SLOT(setNewPandapterRange(int,int)));
    connect(audioOptions, SIGNAL(newWaterfallRange(int,int)), this, SLOT(setNewWaterfallRange(int,int)));
    connect(audioOptions, SIGNAL(newRecDirSelected(QString)), this, SLOT(setNewRecDir(QString)));
    connect(audioOptions, SIGNAL(newUdpHost(QString)), this, SLOT(setNewUdpHost(QString)));
    connect(audioOptions, SIGNAL(newUdpPort(int)), this, SLOT(setNewUdpPort(int)));

    ui->burstSpectrum->setFreqUnits(1000);
    ui->burstSpectrum->setSampleRate(48000);  // Full bandwidth
    ui->burstSpectrum->setSpanFreq(12000);
    ui->burstSpectrum->setCenterFreq(0);
    ui->burstSpectrum->setPercent2DScreen(100);
    ui->burstSpectrum->setFftCenterFreq(6000);
    ui->burstSpectrum->setDemodCenterFreq(0);
    ui->burstSpectrum->setFilterBoxEnabled(false);
    ui->burstSpectrum->setCenterLineEnabled(false);
    ui->burstSpectrum->setBookmarksEnabled(false);
    ui->burstSpectrum->setFftRange(-80., 0.);
    ui->burstSpectrum->setVdivDelta(40);
    ui->burstSpectrum->setHdivDelta(40);
    ui->burstSpectrum->setFreqDigits(1);
    ui->burstSpectrum->setPeakHold(true);
}

DockBurst::~DockBurst()
{
    delete ui;
}

void DockBurst::setFftRange(quint64 minf, quint64 maxf)
{
    if (autoSpan)
    {
        qint32 span = (qint32)(maxf - minf);
        quint64 fc = minf + (maxf - minf)/2;

        ui->burstSpectrum->setFftCenterFreq(fc);
        ui->burstSpectrum->setSpanFreq(span);
        ui->burstSpectrum->setCenterFreq(0);
    }
}

void DockBurst::setSampleRate(float Rate)
{
    ui->burstSpectrum->setSampleRate(Rate);
}

void DockBurst::ResetPeakHold()
{
    ui->burstSpectrum->setPeakHold(true);
}

void DockBurst::setNewFttData(float *fftData, int size)
{
    ui->burstSpectrum->setNewFttData(fftData, size);
}

/*! Set FFT plot color. */
void DockBurst::setFftColor(QColor color)
{
    ui->burstSpectrum->setFftPlotColor(color);
}

/*! Enable/disable filling area under FFT plot. */
void DockBurst::setFftFill(bool enabled)
{
    ui->burstSpectrum->setFftFill(enabled);
}

/*! Public slot to trig audio recording by external events (e.g. satellite AOS).
 *
 * If a recording is already in progress we ignore the event.
 */
void DockBurst::startBurstRecorder(void)
{
    if (ui->burstRecButton->isChecked())
    {
        qDebug() << __func__ << "An burst recording is already in progress";
        return;
    }

    // emulate a button click
    ui->burstRecButton->click();
}

/*! Public slot to stop burst recording by external events (e.g. satellite LOS).
 *
 * The event is ignored if no audio recording is in progress.
 */
void DockBurst::stopBurstRecorder(void)
{

    if (ui->burstRecButton->isChecked())
        ui->burstRecButton->click(); // emulate a button click
    else
        qDebug() << __func__ << "No audio recording in progress";
}

/*! Public slot to set new RX frequency in Hz. */
void DockBurst::setRxFrequency(qint64 freq)
{
    rx_freq = freq;
}


/*! \brief Streaming button clicked.
 *  \param checked Whether streaming is ON or OFF.
 */
void DockBurst::on_audioStreamButton_clicked(bool checked)
{
    if (checked)
        emit audioStreamingStarted(udp_host, udp_port);
    else
        emit audioStreamingStopped();
}

/*! \brief Record button clicked.
 *  \param checked Whether recording is ON or OFF.
 *
 * We use the clicked signal instead of the toggled which allows us to change the
 * state programatically using toggle() without triggering the signal.
 */
void DockBurst::on_burstRecButton_clicked(bool checked)
{
    if (checked)
    {
        // FIXME: option to use local time
        // use toUTC() function compatible with older versions of Qt.
        QString file_name = QDateTime::currentDateTime().toUTC().toString("gqrx_yyyyMMdd_hhmmss");
        last_burst = QString("%1/%2_%3.wav").arg(rec_dir).arg(file_name).arg(rx_freq);

        // emit signal and start timer
        emit burstRecStarted(last_burst);

        ui->burstRecButton->setToolTip(tr("Stop burst recordering"));
    }
    else
    {
        ui->burstRecButton->setToolTip(tr("Start burst recording"));
        emit burstRecStopped();
    }
}


/*! \brief Configure button clicked. */
void DockBurst::on_audioConfButton_clicked()
{
    audioOptions->show();
}

/*! \brief Set status of burst record button. */
void DockBurst::setBurstRecButtonState(bool checked)
{
    if (checked == ui->burstRecButton->isChecked())
    {
        /* nothing to do */
        return;
    }

    // toggle the button and set the state of the other buttons accordingly
    ui->burstRecButton->toggle();
    bool isChecked = ui->burstRecButton->isChecked();

    ui->burstRecButton->setToolTip(isChecked ? tr("Stop burst recorder") : tr("Start burst recorder"));
    //ui->burstRecConfButton->setEnabled(!isChecked);
}


void DockBurst::on_squelchThreshold_valueChanged(double threshold)
{
    printf("blaaaaaaaaa\n");
    emit squelchThresholdChanged(threshold);
}


void DockBurst::saveSettings(QSettings *settings)
{
    int     ival, fft_min, fft_max;

    if (!settings)
        return;

    settings->beginGroup("burst");

    ival = audioOptions->getFftSplit();
    if (ival >= 0 && ival < 100)
        settings->setValue("fft_split", ival);
    else
        settings->remove("fft_split");

    audioOptions->getPandapterRange(&fft_min, &fft_max);
    if (fft_min != -80)
        settings->setValue("pandapter_min_db", fft_min);
    else
        settings->remove("pandapter_min_db");
    if (fft_max != 0)
        settings->setValue("pandapter_max_db", fft_max);
    else
        settings->remove("pandapter_max_db");

    audioOptions->getWaterfallRange(&fft_min, &fft_max);
    if (fft_min != -80)
        settings->setValue("waterfall_min_db", fft_min);
    else
        settings->remove("waterfall_min_db");
    if (fft_max != 0)
        settings->setValue("waterfall_max_db", fft_max);
    else
        settings->remove("waterfall_max_db");

    if (rec_dir != QDir::homePath())
        settings->setValue("rec_dir", rec_dir);
    else
        settings->remove("rec_dir");

    if (udp_host.isEmpty())
        settings->remove("udp_host");
    else
        settings->setValue("udp_host", udp_host);

    if (udp_port != 7355)
        settings->setValue("udp_port", udp_port);
    else
        settings->remove("udp_port");

    settings->endGroup();
}

void DockBurst::readSettings(QSettings *settings)
{
    int     ival, fft_min, fft_max;
    bool    conv_ok = false;

    if (!settings)
        return;

    settings->beginGroup("burst");

    ival = settings->value("fft_split", QVariant(100)).toInt(&conv_ok);
    if (conv_ok)
        audioOptions->setFftSplit(ival);

    fft_min = settings->value("pandapter_min_db", QVariant(-80)).toInt(&conv_ok);
    if (!conv_ok)
        fft_min = -80;
    fft_max = settings->value("pandapter_max_db", QVariant(0)).toInt(&conv_ok);
    if (!conv_ok)
        fft_max = 0;
    audioOptions->setPandapterRange(fft_min, fft_max);

    fft_min = settings->value("waterfall_min_db", QVariant(-80)).toInt(&conv_ok);
    if (!conv_ok)
        fft_min = -80;
    fft_max = settings->value("waterfall_max_db", QVariant(0)).toInt(&conv_ok);
    if (!conv_ok)
        fft_max = 0;
    audioOptions->setWaterfallRange(fft_min, fft_max);

    // Location of burst recordings
    rec_dir = settings->value("rec_dir", QDir::homePath()).toString();
    audioOptions->setRecDir(rec_dir);

    // Audio streaming host and port
    udp_host = settings->value("udp_host", "localhost").toString();
    udp_port = settings->value("udp_port", 7355).toInt(&conv_ok);
    if (!conv_ok)
        udp_port = 7355;

    audioOptions->setUdpHost(udp_host);
    audioOptions->setUdpPort(udp_port);

    settings->endGroup();
}

void DockBurst::setNewPandapterRange(int min, int max)
{
    ui->burstSpectrum->setPandapterRange(min, max);
}

void DockBurst::setNewWaterfallRange(int min, int max)
{
    ui->burstSpectrum->setWaterfallRange(min, max);
}

/*! \brief Slot called when a new valid recording directory has been selected
 *         in the audio conf dialog.
 */
void DockBurst::setNewRecDir(const QString &dir)
{
    rec_dir = dir;
}

/*! \brief Slot called when a new network host has been entered. */
void DockBurst::setNewUdpHost(const QString &host)
{
    if (host.isEmpty())
        udp_host = "localhost";
    else
        udp_host = host;
}

/*! \brief Slot called when a new network port has been entered. */
void DockBurst::setNewUdpPort(int port)
{
    udp_port = port;
}
