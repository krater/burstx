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
#ifndef RECEIVER_H
#define RECEIVER_H

#include <gnuradio/analog/sig_source_c.h>
#include <gnuradio/blocks/file_sink.h>
#include <gnuradio/blocks/multiply_const_ff.h>
#include <gnuradio/blocks/multiply_cc.h>
#include <gnuradio/blocks/null_sink.h>
#include <gnuradio/blocks/wavfile_sink.h>
#include <gnuradio/blocks/wavfile_source.h>
#include <gnuradio/blocks/copy.h>
#include <gnuradio/analog/pwr_squelch_cc.h>
#include <gnuradio/filter/freq_xlating_fir_filter_ccf.h>
#include <gnuradio/top_block.h>
#include <osmosdr/source.h>
#include <string>

#include "dsp/correct_iq_cc.h"
#include "dsp/rx_filter.h"
#include "dsp/rx_meter.h"
#include "dsp/rx_fft.h"
#include "dsp/sniffer_f.h"
#include "dsp/burstfilesink_c.h"
#include "receivers/receiver_base.h"


/**
 * @defgroup DSP Digital signal processing library based on GNU Radio
 */

/**
 * @brief Top-level receiver class.
 * @ingroup DSP
 *
 * This class encapsulates the GNU Radio flow graph for the receiver.
 * Front-ends should only control the receiver through the interface provided
 * by this class.
 */
class receiver
{

public:

    /** Flag used to indicate success or failure of an operation */
    enum status {
        STATUS_OK    = 0, /*!< Operation was successful. */
        STATUS_ERROR = 1  /*!< There was an error. */
    };

    /** Supported receiver types. */
    enum rx_chain {
        RX_CHAIN_NONE  = 0,   /*!< No receiver, just spectrum analyzer. */
        RX_CHAIN_NBRX  = 1,   /*!< Narrow band receiver (AM, FM, SSB). */
        RX_CHAIN_WFMRX = 2    /*!< Wide band FM receiver (for broadcast). */
    };

    /** Filter shape (convenience wrappers for "transition width"). */
    enum filter_shape {
        FILTER_SHAPE_SOFT = 0,   /*!< Soft: Transition band is TBD of width. */
        FILTER_SHAPE_NORMAL = 1, /*!< Normal: Transition band is TBD of width. */
        FILTER_SHAPE_SHARP = 2   /*!< Sharp: Transition band is TBD of width. */
    };

    receiver(const std::string input_device="", unsigned int decimation=1);
    ~receiver();

    void        start();
    void        stop();
    void        set_input_device(const std::string device);

    std::vector<std::string> get_antennas(void) const;
    void        set_antenna(const std::string &antenna);

    double      set_input_rate(double rate);
    double      get_input_rate(void) const { return d_input_rate; }

    unsigned int    set_input_decim(unsigned int decim);
    unsigned int    get_input_decim(void) const { return d_decim; }

    double      get_quad_rate(void) const {
        return d_input_rate / (double)d_decim;
    }

    double      set_analog_bandwidth(double bw);
    double      get_analog_bandwidth(void) const;

    void        set_iq_swap(bool reversed);
    bool        get_iq_swap(void) const;

    void        set_dc_cancel(bool enable);
    bool        get_dc_cancel(void) const;

    void        set_iq_balance(bool enable);
    bool        get_iq_balance(void) const;

    status      set_rf_freq(double freq_hz);
    double      get_rf_freq(void);
    status      get_rf_range(double *start, double *stop, double *step);

    std::vector<std::string>    get_gain_names();
    status      get_gain_range(std::string &name, double *start, double *stop,
                               double *step) const;
    status      set_auto_gain(bool automatic);
    status      set_gain(std::string name, double value);
    double      get_gain(std::string name) const;

    status      set_filter_offset(double offset_hz);
    double      get_filter_offset(void) const;
    status      set_cw_offset(double offset_hz);
    double      get_cw_offset(void) const;
    status      set_filter(double low, double high, filter_shape shape);
    status      set_freq_corr(double ppm);
    float       get_signal_pwr(bool dbfs) const;
    void        set_iq_fft_size(int newsize);
    void        get_iq_fft_data(std::complex<float>* fftPoints,
                                unsigned int &fftsize);
    void        get_burst_fft_data(std::complex<float>* fftPoints,
                                   unsigned int &fftsize);

    /* Squelch parameter */
    status      set_sql_level(double level_db);
    status      set_sql_alpha(double alpha);


    /* Audio parameters */
    status      start_audio_recording(const std::string filename);
    status      stop_audio_recording();
    status      start_audio_playback(const std::string filename);
    status      stop_audio_playback();

    status      start_udp_streaming(const std::string host, int port);
    status      stop_udp_streaming();

    /* sample sniffer */
    status      start_sniffer(unsigned int samplrate, int buffsize);
    status      stop_sniffer();
    void        get_sniffer_data(float * outbuff, unsigned int &num);

    bool        is_recording_audio(void) const { return d_recording_wav; }
    bool        is_snifffer_active(void) const { return d_sniffer_active; }

private:
    void        connect_all();

private:
    bool        d_running;          /*!< Whether receiver is running or not. */
    double      d_input_rate;       /*!< Input sample rate. */
    double      d_quad_rate;        /*!< Quadrature rate (input_rate / decim) */
    unsigned int    d_decim;        /*!< input decimation. */
    double      d_rf_freq;          /*!< Current RF frequency. */
    double      d_filter_offset;    /*!< Current filter offset */
    double      d_cw_offset;        /*!< CW offset */
    bool        d_recording_iq;     /*!< Whether we are recording I/Q file. */
    bool        d_recording_wav;    /*!< Whether we are recording WAV file. */
    bool        d_sniffer_active;   /*!< Only one data decoder allowed. */
    bool        d_iq_rev;           /*!< Whether I/Q is reversed or not. */
    bool        d_dc_cancel;        /*!< Enable automatic DC removal. */
    bool        d_iq_balance;       /*!< Enable automatic IQ balance. */

    std::string input_devstr;  /*!< Current input device string. */
    std::string output_devstr; /*!< Current output device string. */


    gr::top_block_sptr         tb;        /*!< The GNU Radio top block. */

    osmosdr::source::sptr     src;       /*!< Real time I/Q source. */
    dc_corr_cc_sptr           dc_corr;   /*!< DC corrector block. */
    iq_swap_cc_sptr           iq_swap;   /*!< I/Q swapping block. */

    rx_fft_c_sptr             iq_fft;     /*!< Baseband FFT block. */
    rx_fft_c_sptr             burst_fft;  /*!< Burst FFT block. */

    gr::blocks::copy::sptr    copy_in;     /*!< Copy-Block before filter. */
    gr::blocks::copy::sptr    copy_out;    /*!< Copy-Block after filter. */

    gr::analog::pwr_squelch_cc::sptr              squelch;      /*!< Squelch block. */
    burstfilesink_c::sptr                         burstsink;    /*!< Burst sink. */
    gr::filter::freq_xlating_fir_filter_ccf::sptr filter;       /*!< Freq Xlating filter block. */

    sniffer_f_sptr    sniffer;    /*!< Sample sniffer for data decoders. */

    //! Get a path to a file containing random bytes
    static std::string get_random_file(void);

    //! Get a path to a file containing all-zero bytes
    static std::string get_null_file(void);
};

#endif // RECEIVER_H
