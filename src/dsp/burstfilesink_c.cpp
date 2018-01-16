#include <gnuradio/io_signature.h>
#include <stdio.h>
#include <string.h>
#include "burstfilesink_c.h"

//#define ISDELTA(x, d) (((x)+(d)<0.0)||((x)-(d)>0.0))


bool m_open=false;
int  cnt=0;
FILE *m_file=0;
float m_delta=0.5;
int ecnt=0;


burstfilesink_c::sptr burstfilesink_c::make(
        const char *filename,
        int data_threshold,
        bool continues,
        bool active,
        float sample_rate,
        float frequency)
{
  return gnuradio::get_initial_sptr
    (new burstfilesink_c(filename, data_threshold, continues, active, sample_rate, frequency));
}

/*
 * The private constructor
 */
burstfilesink_c::burstfilesink_c(
        const char *filename,
        int data_threshold,
        bool continues,
        bool active,
        float sample_rate,
        float frequency
    ) : gr::sync_block("burstfilesink_c", gr::io_signature::make(1, 1, sizeof(gr_complex)), gr::io_signature::make(0, 0, 0)),
          m_filename(filename),
          m_data_threshold(data_threshold),
          m_continues(continues),
          m_active(active),
          m_sample_rate(sample_rate),
          m_frequency(frequency)
{
}

/*
 * Our virtual destructor.
 */
burstfilesink_c::~burstfilesink_c()
{
    if(m_open)
    {
        fclose(m_file);
    }
}

int burstfilesink_c::work(
            int noutput_items,
            gr_vector_const_void_star &input_items,
            gr_vector_void_star &output_items
)
{
    const gr_complex *in = (const gr_complex *) input_items[0];
    int offset=0;
    for(int i=0;i<noutput_items;i++)
    {
        if(((in[i].real()==0.0) && (in[i].imag()==0.0)) || !m_active) //yes, I'm serios
        {
            ecnt++;
            if(m_open&&(ecnt > m_data_threshold))
            {
                fwrite(in+offset, i-offset-m_data_threshold, sizeof(gr_complex), m_file);
                fclose(m_file);
                m_open=false;
                offset=i;
            }
        }
        else
        {
            ecnt=0;
            if(!m_open)
            {
                char fn[1024];
                snprintf(fn, sizeof(fn), "%s%u.wav", m_filename, cnt);
                cnt++;

                printf("New File:%s\n", fn);
                m_file = fopen(fn, "wb");
                m_open=true;
                WriteWavHeader();
            }
        }
    }

    if(m_open)
    {
        fwrite(in+offset, noutput_items-offset, sizeof(gr_complex), m_file);
    }


    return noutput_items;
}

void burstfilesink_c::WriteWavHeader()
{
    uint8_t RIFF_Chunk[]={'R', 'I', 'F', 'F', 0xff, 0xff, 0xff, 0xff, 'W', 'A', 'V', 'E'};
    uint8_t FMT_Chunk[]={'f','m','t',' ',   // Magic value
                         0x28, 0, 0, 0,     // length fmt-chunk
                         0xfe, 0xff,        // format code: PCM
                         0x02, 0x00,        // Channel count
                         0x40, 0x1f, 0x00, 0x00,        // samplerate
                         0x00, 0xfa, 0x00, 0x00,        // bps
                         0x08, 0x00,        // Byte per sample
                         32,0,              // Bits per sample
                         0x16,0,
                         32, 0,
                         3,0,0,0,
                         3,0,0,0,
                         0,0,0x10,0,
                         0x80,0,0,0xaa,0,0x38,0x9b,0x71
                        };

    uint8_t Data_Chunk[]={'d','a','t','a', 0xff, 0xff, 0xff, 0xff};

    fwrite(RIFF_Chunk, 1, sizeof(RIFF_Chunk), m_file);
    fwrite(FMT_Chunk,  1, sizeof(FMT_Chunk), m_file);
    fwrite(Data_Chunk, 1, sizeof(Data_Chunk), m_file);
}

