#ifndef BURSTFILESINK_C_H
#define BURSTFILESINK_C_H

#include <gnuradio/sync_block.h>

class burstfilesink_c : public gr::sync_block
{
public:
    typedef boost::shared_ptr<burstfilesink_c> sptr;

    static sptr make(const char *filename="Burst_", int data_threshold=300, bool continues=false, bool active=true, float sample_rate=0, float frequency=0);

    burstfilesink_c(const char *filename, int data_threshold, bool continues, bool active, float sample_rate, float frequency);
    ~burstfilesink_c();

    // Where all the action really happens
    int work(int noutput_items,
        gr_vector_const_void_star &input_items,
        gr_vector_void_star &output_items);

    void set_active(bool active);


private:
    const char *m_filename;
    int   m_data_threshold;
    bool  m_continues;
    bool  m_active;
    float m_sample_rate;
    float m_frequency;

    void WriteWavHeader();
};



#endif // BURSTFILESINK_C_H
