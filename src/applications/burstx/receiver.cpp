/* -*- c++ -*- */
/*
 * Gqrx SDR: Software defined radio receiver powered by GNU Radio and Qt
 *           http://gqrx.dk/
 *
 * Copyright 2011-2014 Alexandru Csete OZ9AEC.
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
#include <cmath>
#include <iostream>
#ifndef _MSC_VER
#include <unistd.h>
#endif

#include <iostream>

#include <gnuradio/blocks/multiply_const_ff.h>
#include <gnuradio/prefs.h>
#include <gnuradio/top_block.h>
#include <osmosdr/source.h>
#include <osmosdr/ranges.h>

#include "applications/gqrx/receiver.h"
#include "dsp/correct_iq_cc.h"
#include "dsp/filter/fir_decim.h"
#include "dsp/rx_fft.h"
#include "dsp/burstfilesink_c.h"
#include "receivers/nbrx.h"
#include "receivers/wfmrx.h"



/**
 * @brief Public contructor.
 * @param input_device Input device specifier.
 * @param audio_device Audio output device specifier,
 *                     e.g. hw:0 when using ALSA or Portaudio.
 */
receiver::receiver(const std::string input_device, unsigned int decimation)
    : d_running(false),
      d_input_rate(3000000.0),
      d_decim(decimation),
      d_rf_freq(144800000.0),
      d_filter_offset(0.0),
      d_cw_offset(0.0),
      d_recording_iq(false),
      d_recording_wav(false),
      d_sniffer_active(false),
      d_iq_rev(false),
      d_dc_cancel(false),
      d_iq_balance(false)
{

    tb = gr::make_top_block("gqrx");

    if (input_device.empty())
    {
        src = osmosdr::source::make("file="+get_random_file()+",freq=428e6,rate=96000,repeat=true,throttle=true");
    }
    else
    {
        input_devstr = input_device;
        src = osmosdr::source::make(input_device);
    }

    iq_swap = make_iq_swap_cc(false);
    dc_corr = make_dc_corr_cc(d_input_rate, 1.0);
    iq_fft = make_rx_fft_c(8192u, 0);

    std::vector<float> taps;
    taps=gr::filter::firdes::low_pass(10/*gain*/, d_input_rate, 1000000/*cutof freq*/, 100000/*transition width*/, gr::filter::firdes::WIN_HAMMING, 6.76);
    filter = gr::filter::freq_xlating_fir_filter_ccf::make(d_decim, taps, d_filter_offset/*d_rf_freq/*centerfreq...freq shift!!!!*/, d_input_rate/*sampling freq*/);

    copy_in  = gr::blocks::copy::make(sizeof(gr_complex));
    copy_out = gr::blocks::copy::make(sizeof(gr_complex));

    squelch = gr::analog::pwr_squelch_cc::make(1/*db*/, 0.0001/*alpha*/, 0, false);
    burstsink = burstfilesink_c::make();

    burst_fft = make_rx_fft_c(8192u);

    sniffer = make_sniffer_f();
    /* sniffer_rr is created at each activation. */
    connect_all();

#ifndef QT_NO_DEBUG_OUTPUT
    gr::prefs pref;
#endif
}

receiver::~receiver()
{
    tb->stop();
}


/** Convenience function to connect all blocks. */
void receiver::connect_all()
{
    // Allow reconf using same demod to provide a workaround
    // for the "jerky streaming" we may experience with rtl
    // dongles (the jerkyness disappears when we run this function)
    //if (demod == d_demod)
    //    return ret;

    // tb->lock() seems to hang occasioanlly
    if (d_running)
    {
        tb->stop();
        tb->wait();
    }

    tb->disconnect_all();


    printf("1");
    // Connect all
    tb->connect(src, 0, iq_swap, 0);

    if (d_dc_cancel)
    {
        tb->connect(iq_swap, 0, dc_corr, 0);
        tb->connect(dc_corr, 0, copy_in, 0);
    }
    else
    {
        tb->connect(iq_swap, 0, copy_in, 0);
    }

    printf("2");
    tb->connect(copy_in, 0, iq_fft, 0);
    tb->connect(copy_in, 0, filter, 0);
    printf("3");
    tb->connect(filter, 0, copy_out, 0);

    tb->connect(copy_out, 0, squelch, 0);
    tb->connect(copy_out, 0, burst_fft, 0);
    printf("4");
    tb->connect(squelch, 0, burstsink, 0);

/*
    printf("5");
    if (d_sniffer_active)
    {
        tb->connect(rx, 0, sniffer_rr, 0);
        tb->connect(sniffer_rr, 0, sniffer, 0);
    }
*/
    if (d_running)
        tb->start();

    return;
}

/** Start the receiver. */
void receiver::start()
{
    if (!d_running)
    {
        tb->start();
        d_running = true;
    }
}

/** Stop the receiver. */
void receiver::stop()
{
    if (d_running)
    {
        tb->stop();
        tb->wait(); // If the graph is needed to run again, wait() must be called after stop
        d_running = false;
    }
}

/**
 * @brief Select new input device.
 *
 * @bug When using ALSA, program will crash if the new device
 *      is the same as the previously used device:
 *      audio_alsa_source[hw:1]: Device or resource busy
 */
void receiver::set_input_device(const std::string device)
{
    if (device.empty())
        return;

    if (input_devstr.compare(device) == 0)
    {
#ifndef QT_NO_DEBUG_OUTPUT
        std::cout << "No change in input device:" << std::endl
                  << "  old: " << input_devstr << std::endl
                  << "  new: " << device << std::endl;
#endif
        return;
    }

    input_devstr = device;

    // tb->lock() can hang occasionally
    if (d_running)
    {
        tb->stop();
        tb->wait();
    }

    tb->disconnect(src, 0, iq_swap, 0);

    src.reset();
    src = osmosdr::source::make(device);
    if(src->get_sample_rate() != 0)
        set_input_rate(src->get_sample_rate());

    tb->connect(src, 0, iq_swap, 0);

    if (d_running)
        tb->start();
}


/** Get a list of available antenna connectors. */
std::vector<std::string> receiver::get_antennas(void) const
{
    return src->get_antennas();
}

/** Select antenna conenctor. */
void receiver::set_antenna(const std::string &antenna)
{
    if (!antenna.empty())
    {
        src->set_antenna(antenna);
    }
}

/**
 * @brief Set new input sample rate.
 * @param rate The desired input rate
 * @return The actual sample rate set or 0 if there was an error with the
 *         device.
 */
double receiver::set_input_rate(double rate)
{
    double  current_rate;
    bool    rate_has_changed;

    current_rate = src->get_sample_rate();
    rate_has_changed = !(rate == current_rate ||
            std::abs(rate - current_rate) < std::abs(std::min(rate, current_rate))
            * std::numeric_limits<double>::epsilon());

    tb->lock();
    d_input_rate = src->set_sample_rate(rate);

    if (d_input_rate == 0)
    {
        // This can be the case when no device is attached and gr-osmosdr
        // puts in a null_source with rate 100 ksps or if the rate has not
        // changed
        if (rate_has_changed)
        {
            std::cerr << std::endl;
            std::cerr << "Failed to set RX input rate to " << rate << std::endl;
            std::cerr << "Your device may not be working properly." << std::endl;
            std::cerr << std::endl;
        }
        d_input_rate = rate;
    }

    dc_corr->set_sample_rate(d_input_rate);

    //Is this really needet ? Or is there a set_samplerate anywhere ?
/*    tb->disconnect(iq_swap, 0, filter, 0);
    tb->disconnect(filter, 0, squelch, 0);
    tb->disconnect(filter, 0, burst_fft, 0);
*/
    tb->disconnect(copy_in, 0, filter, 0);
    tb->disconnect(filter, 0, copy_out, 0);

    filter.reset();

    std::vector<float> taps;
    try
    {
        taps=gr::filter::firdes::low_pass(10/*gain*/, d_input_rate, 1000000/*cutof freq*/, 100000/*transition width*/, gr::filter::firdes::WIN_HAMMING, 6.76);
        printf("generated low_pass\n");
    }
    catch(std::out_of_range msg)
    {
        printf("Failed to generate low_pass...\n");
        taps.push_back(1.0);
    }

    filter = gr::filter::freq_xlating_fir_filter_ccf::make(
                d_decim,
                taps,
                d_filter_offset,
                d_input_rate);

    tb->connect(copy_in, 0, filter, 0);
    tb->connect(filter, 0, copy_out, 0);

/*    tb->connect(filter, 0, squelch, 0);
    tb->connect(filter, 0, burst_fft, 0);
*/
    tb->unlock();

    return d_input_rate;
}

/** Set input decimation */
unsigned int receiver::set_input_decim(unsigned int decim)
{
    printf("input decimation: %d\n",decim);

    if (decim == d_decim)
        return d_decim;

    if (d_running)
    {
        tb->stop();
        tb->wait();
    }

    filter.reset();
    filter->set_decimation(decim);

    d_decim = decim;

#ifdef CUSTOM_AIRSPY_KERNELS
    if (input_devstr.find("airspy") != std::string::npos)
        src->set_bandwidth(d_quad_rate);
#endif

    if (d_running)
        tb->start();

    return d_decim;
}

/**
 * @brief Set new analog bandwidth.
 * @param bw The new bandwidth.
 * @return The actual bandwidth.
 */
double receiver::set_analog_bandwidth(double bw)
{
    printf("Set analog bandwidth\n");
    return src->set_bandwidth(bw);
}

/** Get current analog bandwidth. */
double receiver::get_analog_bandwidth(void) const
{
    printf("get analog bandwidth\n");
    return src->get_bandwidth();
}

/** Set I/Q reversed. */
void receiver::set_iq_swap(bool reversed)
{
    if (reversed == d_iq_rev)
        return;

    d_iq_rev = reversed;
    iq_swap->set_enabled(d_iq_rev);
}

/**
 * @brief Get current I/Q reversed setting.
 * @retval true I/Q swappign is enabled.
 * @retval false I/Q swapping is disabled.
 */
bool receiver::get_iq_swap(void) const
{
    return d_iq_rev;
}

/**
 * @brief Enable/disable automatic DC removal in the I/Q stream.
 * @param enable Whether DC removal should enabled or not.
 */
void receiver::set_dc_cancel(bool enable)
{
    if (enable == d_dc_cancel)
        return;

    if (d_dc_cancel)
    {
        tb->disconnect(iq_swap, 0, dc_corr, 0);
        tb->disconnect(dc_corr, 0, copy_in, 0);
    }
    else
    {
        tb->disconnect(iq_swap, 0, copy_in, 0);
    }


    d_dc_cancel = enable;

    if (d_dc_cancel)
    {
        tb->connect(iq_swap, 0, dc_corr, 0);
        tb->connect(dc_corr, 0, copy_in, 0);
    }
    else
    {
        tb->connect(iq_swap, 0, copy_in, 0);
    }
}

/**
 * @brief Get auto DC cancel status.
 * @retval true  Automatic DC removal is enabled.
 * @retval false Automatic DC removal is disabled.
 */
bool receiver::get_dc_cancel(void) const
{
    printf("get_dc_cancel\n");
    return d_dc_cancel;
}

/**
 * @brief Enable/disable automatic I/Q balance.
 * @param enable Whether automatic I/Q balance should be enabled.
 */
void receiver::set_iq_balance(bool enable)
{
    printf("TODO: set_iq_balance. is doing nothing...\n");
    if (enable == d_iq_balance)
        return;

    d_iq_balance = enable;

    src->set_iq_balance_mode(enable ? 2 : 0);
}

/**
 * @brief Get auto I/Q balance status.
 * @retval true  Automatic I/Q balance is enabled.
 * @retval false Automatic I/Q balance is disabled.
 */
bool receiver::get_iq_balance(void) const
{
    printf("get_iq_balance\n");
    return d_iq_balance;
}

/**
 * @brief Set RF frequency.
 * @param freq_hz The desired frequency in Hz.
 * @return RX_STATUS_ERROR if an error occurs, e.g. the frequency is out of range.
 * @sa get_rf_freq()
 */
receiver::status receiver::set_rf_freq(double freq_hz)
{
    d_rf_freq = freq_hz;

    src->set_center_freq(d_rf_freq);
    // FIXME: read back frequency?

    filter->set_center_freq(d_rf_freq);

    return STATUS_OK;
}

/**
 * @brief Get RF frequency.
 * @return The current RF frequency.
 * @sa set_rf_freq()
 */
double receiver::get_rf_freq(void)
{
    d_rf_freq = src->get_center_freq();

    return d_rf_freq;
}

/**
 * @brief Get the RF frequency range of the current input device.
 * @param start The lower limit of the range in Hz.
 * @param stop  The upper limit of the range in Hz.
 * @param step  The frequency step in Hz.
 * @returns STATUS_OK if the range could be retrieved, STATUS_ERROR if an error has occurred.
 */
receiver::status receiver::get_rf_range(double *start, double *stop, double *step)
{
    osmosdr::freq_range_t range;

    range = src->get_freq_range();

    // currently range is empty for all but E4000
    if (!range.empty())
    {
        if (range.start() < range.stop())
        {
            *start = range.start();
            *stop  = range.stop();
            *step  = range.step();  /** FIXME: got 0 for rtl-sdr? **/

            return STATUS_OK;
        }
    }

    return STATUS_ERROR;
}

/** Get the names of available gain stages. */
std::vector<std::string> receiver::get_gain_names()
{
    return src->get_gain_names();
}

/**
 * @brief Get gain range for a specific stage.
 * @param[in]  name The name of the gain stage.
 * @param[out] start Lower limit for this gain setting.
 * @param[out] stop  Upper limit for this gain setting.
 * @param[out] step  The resolution for this gain setting.
 *
 * This function retunrs the range for the requested gain stage.
 */
receiver::status receiver::get_gain_range(std::string &name, double *start,
                                          double *stop, double *step) const
{
    osmosdr::gain_range_t range;

    range = src->get_gain_range(name);
    *start = range.start();
    *stop  = range.stop();
    *step  = range.step();

    return STATUS_OK;
}

receiver::status receiver::set_gain(std::string name, double value)
{
    src->set_gain(value, name);
    return STATUS_OK;
}

double receiver::get_gain(std::string name) const
{
    return src->get_gain(name);
}

/**
 * @brief Set RF gain.
 * @param gain_rel The desired relative gain between 0.0 and 1.0 (use -1 for
 *                 AGC where supported).
 * @return RX_STATUS_ERROR if an error occurs, e.g. the gain is out of valid range.
 */
receiver::status receiver::set_auto_gain(bool automatic)
{
    src->set_gain_mode(automatic);

    return STATUS_OK;
}

/**
 * @brief Set filter offset.
 * @param offset_hz The desired filter offset in Hz.
 * @return RX_STATUS_ERROR if the tuning offset is out of range.
 *
 * This method sets a new tuning offset for the receiver. The tuning offset is used
 * to tune within the passband, i.e. select a specific channel within the received
 * spectrum.
 *
 * The valid range for the tuning is +/- 0.5 * the bandwidth although this is just a
 * logical limit.
 *
 * @sa get_filter_offset()
 */
receiver::status receiver::set_filter_offset(double offset_hz)
{
    printf("set filter offset:%f\n",offset_hz);
    d_filter_offset = offset_hz;

    filter->set_center_freq(d_filter_offset);

    return STATUS_OK;
}

/**
 * @brief Get filter offset.
 * @return The current filter offset.
 * @sa set_filter_offset()
 */
double receiver::get_filter_offset(void) const
{
    printf("get filter offset\n");
    return d_filter_offset;
}

/* CW offset can serve as a "BFO" if the GUI needs it */
receiver::status receiver::set_cw_offset(double offset_hz)
{
    printf("set cq offset:%u offset_hz\n");
    d_cw_offset = offset_hz;

    return STATUS_OK;
}

double receiver::get_cw_offset(void) const
{
    return d_cw_offset;
}

receiver::status receiver::set_filter(double low, double high, filter_shape shape)
{
    printf("set filter %f %f %u\n",low, high, shape);

    std::vector<float> taps;
    try
    {
        taps=gr::filter::firdes::low_pass(1/*gain*/, d_input_rate, (2*high)/*cutof freq*/, 100000/*transition width*/, gr::filter::firdes::WIN_HAMMING, 6.76);
        //taps.push_back(1.0);
        printf("generated low_pass\n");
    }
    catch(std::out_of_range msg)
    {
        printf("Failed to generate low_pass...\n");
        taps.push_back(1.0);
    }

    filter->set_taps(taps);

    return STATUS_OK;
}

receiver::status receiver::set_freq_corr(double ppm)
{
    printf("set freq coirr\n");
    src->set_freq_corr(ppm);

    return STATUS_OK;
}

/**
 * @brief Get current signal power.
 * @param dbfs Whether to use dbfs or absolute power.
 * @return The current signal power.
 *
 * This method returns the current signal power detected by the receiver. The detector
 * is located after the band pass filter. The full scale is 1.0
 */
float receiver::get_signal_pwr(bool dbfs) const
{
    return 42;
//    return rx->get_signal_level(dbfs);
}

/** Set new FFT size. */
void receiver::set_iq_fft_size(int newsize)
{
    iq_fft->set_fft_size(newsize);
}

/** Get latest baseband FFT data. */
void receiver::get_iq_fft_data(std::complex<float>* fftPoints, unsigned int &fftsize)
{
    iq_fft->get_fft_data(fftPoints, fftsize);
}

/** Get latest audio FFT data. */
void receiver::get_burst_fft_data(std::complex<float>* fftPoints, unsigned int &fftsize)
{
    burst_fft->get_fft_data(fftPoints, fftsize);
}

/**
 * @brief Set squelch level.
 * @param level_db The new level in dBFS.
 */
receiver::status receiver::set_sql_level(double level_db)
{
    printf("set sql level\n");
    squelch->set_threshold(level_db);
    return STATUS_OK; // FIXME
}

/** Set squelch alpha */
receiver::status receiver::set_sql_alpha(double alpha)
{
    printf("set sql alpha\n");
    squelch->set_alpha(alpha);
    return STATUS_OK; // FIXME
}



#if 0
receiver::status receiver::set_af_gain(float gain_db)
{
    float k;

    /* convert dB to factor */
    k = pow(10.0, gain_db / 20.0);
    //std::cout << "G:" << gain_db << "dB / K:" << k << std::endl;


    return STATUS_OK;
}
#endif

/**
 * @brief Start data sniffer.
 * @param buffsize The buffer that should be used in the sniffer.
 * @return STATUS_OK if the sniffer was started, STATUS_ERROR if the sniffer is already in use.
 */
receiver::status receiver::start_sniffer(unsigned int samprate, int buffsize)
{
    /*if (d_sniffer_active) {
        /* sniffer already in use
        return STATUS_ERROR;
    }

    sniffer->set_buffer_size(buffsize);
    sniffer_rr = make_resampler_ff((float)samprate/(float)d_audio_rate);
    tb->lock();
    tb->connect(rx, 0, sniffer_rr, 0);
    tb->connect(sniffer_rr, 0, sniffer, 0);
    tb->unlock();
    d_sniffer_active = true;*/

    return STATUS_OK;
}

/**
 * @brief Stop data sniffer.
 * @return STATUS_ERROR i the sniffer is not currently active.
 */
receiver::status receiver::stop_sniffer()
{
 /*   if (!d_sniffer_active) {
        return STATUS_ERROR;
    }

    tb->lock();
    tb->disconnect(rx, 0, sniffer_rr, 0);
    tb->disconnect(sniffer_rr, 0, sniffer, 0);
    tb->unlock();
    d_sniffer_active = false;

    /* delete resampler
    sniffer_rr.reset();
*/
    return STATUS_OK;
}

/** Get sniffer data. */
void receiver::get_sniffer_data(float * outbuff, unsigned int &num)
{
    sniffer->get_samples(outbuff, num);
}
