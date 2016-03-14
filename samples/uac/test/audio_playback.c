#include <signal.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/types.h>
#include <sys/stat.h>

#include <assert.h>
#include <fcntl.h>           /* For O_* constants */

#include "alsa/asoundlib.h"

#define DEFAULT_CHANNELS_NUM 1

#define MAX_SUPPORTED_CHANNEL_NUM 20

#define INTERLEAVED_MODE 0x0
#define NON_INTERLEAVED_MODE 0x1

//The device file of alsa could be checked through : /proc/asound/cards .
#define DEVFILE "hw:2,0"

static int bTerminate = 0;
static int bSleep = 0;

static void sig_kill(int signo)
{
	printf("Receive SIGNAL: %d\n", signo);
	if ((signo == SIGTERM) || (signo == SIGINT)) bTerminate = 1;
	else if (signo == SIGHUP) {
		if (bSleep == 0) 
			bSleep = 1;
		else 
			bSleep =0;
		//bIssueEvent = 1;
	}
}
#define INTERLEAVED_FILE_NAME "playback"

static unsigned int g_count = (unsigned int)-1;
static unsigned int g_is_splite = (unsigned int)-1;
static unsigned int g_bShowFrameRate = (unsigned int)-1;

static int   playback_func(snd_pcm_t *handle, unsigned int period_num_of_buffer, snd_pcm_uframes_t frame_num_of_period, int channel_number, unsigned int sample_size, FILE **fps, int mode);
#define RETERRIFNEG(x)	if(x<0) { printf("error at %s@%d: %s=%d\n", __func__, __LINE__, #x, x ); return x;}

#define AUDIO_SAMPLING_RATE	48000
#define PCM_NUM_CH			1
#define FIFO_NUM_CH			2

#define AUDIO_BUFFER_PERIOD     10
#define AUDIO_PERIOD_PER_SEC	100
#define AUDIO_PERIOD_MAX_SIZE   (AUDIO_SAMPLING_RATE/AUDIO_PERIOD_PER_SEC)
#define AUDIO_PERIOD_MS         (1000/AUDIO_PERIOD_PER_SEC)
#define AUDIO_PERIOD_SIZE		(AUDIO_SAMPLING_RATE / AUDIO_PERIOD_PER_SEC)

#define PCM_FRAME_BYTES			(PCM_NUM_CH * 2)
#define FIFO_FRAME_BYTES		(FIFO_NUM_CH * 2)

#define PCM_PERIOD_BYTES	(AUDIO_PERIOD_SIZE * PCM_FRAME_BYTES)
#define FIFO_PERIOD_BYTES	(AUDIO_PERIOD_SIZE * FIFO_FRAME_BYTES)
#define AUDIO_BUFFER_SIZE	(AUDIO_PERIOD_SIZE * AUDIO_BUFFER_PERIOD)
#define PLAY_MIN_PERIOD         3
#define AUDIO_ACCESS_MODE       SND_PCM_ACCESS_RW_INTERLEAVED
#define AUDIO_PCM_FORMAT        SND_PCM_FORMAT_S16_LE

int  print_usage(void)
{
	printf(" -m : access mode,\n") ;
	printf("      this can be 0(interleaved), 1(non interleaved)\n");
	printf(" -n : channel number, this should be 1~%d\n", MAX_SUPPORTED_CHANNEL_NUM) ;
	printf(" -t : playback format, this can be : 0(SND_PCM_FORMAT_MU_LAW), 1(SND_PCM_FORMAT_A_LAW), 2(SND_PCM_FORMAT_S16_LE), 3(SND_PCM_FORMAT_S24_LE)\n") ;
	printf(" -d : playback device, this can be : 0:hw:0,0, 1:hw:0,1, 2:hw:1,0, 3:hw:1,1, 4:hw:2,0, 5:hw:2,1, 6:hw:3,0, 7:hw:3,1\n") ;
	printf(" -r : playback sample rate, this can be 8000, 16000, 32000, 44100, 48000.\n") ;
	printf(" -b : dump file or not\n") ;
	printf(" -c : playback count\n") ;
	printf(" -s:  splite channel\n");
	printf(" -S:  show frame rate\n");
	return 0 ;
}

int setSwParams( snd_pcm_t *handle, 
		snd_pcm_uframes_t startFrame,	/* auto start transfer threshold */
		snd_pcm_uframes_t stopFrame,	/* auto stop transfer threshold */
		snd_pcm_uframes_t period_size	/* PCM ready size */
		)
{
	int err;
	snd_pcm_sw_params_t *swparams;

	snd_pcm_sw_params_alloca(&swparams);

	/* get the current swparams */
	err = snd_pcm_sw_params_current(handle, swparams);
	RETERRIFNEG(err);

	/* auto start transfer threshold */
	err = snd_pcm_sw_params_set_start_threshold(handle, swparams, startFrame);
	RETERRIFNEG(err);

	/* auto stop transfer threshold */
	err = snd_pcm_sw_params_set_stop_threshold(handle, swparams, stopFrame);
	RETERRIFNEG(err);

	/* allow the transfer when at least period_size samples can be processed */
	err = snd_pcm_sw_params_set_avail_min(handle, swparams, period_size);
	RETERRIFNEG(err);

	/* write the parameters to the playback device */
	err = snd_pcm_sw_params(handle, swparams);
	RETERRIFNEG(err);

	return 0;
}


int setHwParams(snd_pcm_t *pcm_handle,
		snd_pcm_uframes_t buffer_frames,	/* ring buffer length */
		snd_pcm_uframes_t period_frames,    /* period time */
		unsigned int channels,
		unsigned int rate
		)
{
	int rc;
	unsigned int val;
	int dir;
	snd_pcm_hw_params_t* params;

	/* Allocate a hardware parameters object */
	snd_pcm_hw_params_alloca(&params);

	/* First set to default values */
	rc = snd_pcm_hw_params_any(pcm_handle, params);
	RETERRIFNEG(rc);

	/* Set the desired hardware parameters. */

	/* Set access mode - Interleaved/non-interleaved */
	rc = snd_pcm_hw_params_set_access(pcm_handle, params, AUDIO_ACCESS_MODE);
	RETERRIFNEG(rc);

	/* Set PCM format */
	rc = snd_pcm_hw_params_set_format(pcm_handle, params, AUDIO_PCM_FORMAT);
	RETERRIFNEG(rc);

	/* Set # channels, 1 -> mono, 2 -> stereo */
	rc = snd_pcm_hw_params_set_channels(pcm_handle, params, channels);
	RETERRIFNEG(rc);

	/* Set sampling rate */
	val = rate;
	dir = 0;
	rc = snd_pcm_hw_params_set_rate_near(pcm_handle, params, &val, &dir);
	RETERRIFNEG(rc);
	if( val != rate)
		printf("warnning: rate is not accurate, want %d Hz, got %d Hz\n", rate, val);

	if( buffer_frames)
	{
		/* set the buffer size */
		rc = snd_pcm_hw_params_set_buffer_size_near(pcm_handle, params, &buffer_frames);
		RETERRIFNEG(rc);
	}

	if( period_frames)
	{
		/* set the period size */
		rc = snd_pcm_hw_params_set_period_size_near(pcm_handle, params, &period_frames, &dir);
		RETERRIFNEG(rc);
	}

	/* Write the parameters to the driver */
	rc = snd_pcm_hw_params(pcm_handle, params);
	RETERRIFNEG(rc);

	return 0;
}

static int xrun_recovery(snd_pcm_t *handle, int err)
{
	printf("stream recovery from ...");
	printf("xrun_recovery: %s xrun, err %d", 
			(snd_pcm_stream(handle) == SND_PCM_STREAM_PLAYBACK) ? "playback" : "capture", err);

	if (err == -EPIPE) 
	{
		if (snd_pcm_stream(handle) == SND_PCM_STREAM_PLAYBACK)
			printf("underrun...");
		else
			printf("overrun...");

		err = snd_pcm_prepare(handle);
		if (err < 0)
		{
			printf("Can't recovery from underrun, prepare failed: %s\n", snd_strerror(err));
			return err;
		}
		printf("ready \n");
		return 0;
	}
	else if (err == -ESTRPIPE) 
	{
		int sleepcnt=0;
		printf("suspend...");
		while ((err = snd_pcm_resume(handle)) == -EAGAIN)
		{
			sleep(1);       /* wait until the suspend flag is released */
			if(++sleepcnt > 5)
				break;
		}

		if (err < 0) 
		{
			err = snd_pcm_prepare(handle);
			if (err < 0)
			{
				printf("Can't recovery from suspend, prepare failed: %s\n", snd_strerror(err));
				return err;
			}
		}
		printf("ready \n");
		return 0;
	}

	printf("unknown \n");
	return err;
}



int main(int argc, char* argv[]) {
	int rc;

	int access_mode = INTERLEAVED_MODE ;

	int playback_format = 0 ;

	snd_pcm_uframes_t frame_num_of_period  = 0;
	unsigned int  period_num_of_buffer = 0; // number of periods per buffer

	int sample_size = 0 ;
	int frame_size   = 0;
	int period_size  = 0;
	int buffer_size  = 0;

	snd_pcm_t *handle;
	char *pcm_name;

	snd_pcm_hw_params_t *params;
	snd_pcm_format_t sample_format;

	unsigned int val = 0;
	int dir = 0;
	int channel_number = -1 ;

	int flag = 0;
	FILE *fps = NULL;

	unsigned int samplerate = 8000;

	char file_name[128] = {0x0};

	unsigned int dump_file = 0;
	short *buffers = NULL;
	short *zeroBuf = NULL;
	int err = 0;
	int count=0;
	int size = PCM_PERIOD_BYTES;
	int n = AUDIO_BUFFER_SIZE * 2;  

	buffers = (short *) malloc(size);  
	memset(buffers, 0x0, size);

	zeroBuf = (short *) malloc(n);
	memset(zeroBuf, 0, n);


	pcm_name =  strdup(DEVFILE);
	while ((flag=getopt(argc, argv, "n:m:t:d:r:b:c:s:S:h"))!=-1) 
	{
		switch (flag) 
		{
			case 'm':
				switch (optarg[0]) 
				{
					case '0':
						access_mode = INTERLEAVED_MODE;
						break;
					case '1':
						access_mode = NON_INTERLEAVED_MODE; // non interleaved
						break;
					default:
						access_mode= INTERLEAVED_MODE;
				}
				break;				
			case 'n':
				channel_number = (unsigned int)atoi(optarg);
				break ;
			case 't': 
				switch (optarg[0]) 
				{
					case '0':// SND_PCM_FORMAT_MU_LAW 
						playback_format =0;
						break;
					case '1':// SND_PCM_FORMAT_A_LAW
						playback_format = 1; 
						break;
					case '2'://SND_PCM_FORMAT_S16_LE 
						playback_format = 2; 
						break;
					case '3'://SND_PCM_FORMAT_S24_LE (Signed 24 bit Little Endian using low three bytes in 32-bit word)
						playback_format = 3;
						break;
					default:
						playback_format = 0 ;
				}
				break;
			case 'd': // device
				/* Open PCM device for recording (playback). */
				switch (optarg[0]) 
				{
					case '0':
						pcm_name = strdup("hw:0,0");
						break;
					case '1':
						pcm_name = strdup("hw:0,1");
						break;
					case '2':
						pcm_name = strdup("hw:1,0");
						break;
					case '3':
						pcm_name = strdup("hw:1,1");
						break;
					case '4':
						pcm_name = strdup("hw:2,0");
						break;
					case '5':
						pcm_name = strdup("hw:2,1");
						break;
					case '6':
						pcm_name = strdup("hw:3,0");
						break;	
					case '7':
						pcm_name = strdup("hw:3,1");
						break;		
					default:
						pcm_name = strdup("hw:0,0");
				}
				break;
			case 'r':
				samplerate = (unsigned int)atoi(optarg);
				break;
			case 'b':
				dump_file = (unsigned int)atoi(optarg);
				break;
			case 'c':
				g_count  = 	(unsigned int)atoi(optarg);
				break;
			case 's':
				g_is_splite = (unsigned int)atoi(optarg);
				break;
			case 'S':
				g_bShowFrameRate=  (unsigned int)atoi(optarg);
				break;
			case 'h' :
			default:
				print_usage();
				exit(0);
				break;
		}
	}

	if (channel_number == -1) {
		print_usage();
		exit(0);
	}

	signal(SIGTERM, sig_kill);
	signal(SIGINT, sig_kill);
	signal(SIGHUP, sig_kill);

	fps = fopen ( "/root/playback.pcm" , "rb" );
	/* Open PCM device for playback. */
	rc = snd_pcm_open(&handle, pcm_name, SND_PCM_STREAM_PLAYBACK, 0);
	if (rc < 0) 
	{
		fprintf(stderr,	"[audio_playback] unable to open pcm device: %s\n", snd_strerror(rc));
		exit(1);
	}

	RETERRIFNEG(rc);
	rc = setHwParams(handle, AUDIO_BUFFER_SIZE, AUDIO_PERIOD_SIZE, PCM_NUM_CH, AUDIO_SAMPLING_RATE);
	RETERRIFNEG(rc);
	rc = setSwParams(handle, AUDIO_PERIOD_SIZE, AUDIO_BUFFER_SIZE*2, AUDIO_PERIOD_SIZE);
	RETERRIFNEG(rc);
	count = 0;
	while(fps!=NULL && !feof(fps))
	{
		if (bTerminate == 1) goto exit;
		if(fps!=NULL)
			rc = fread(buffers, sizeof(short), AUDIO_PERIOD_SIZE, fps);
		RETERRIFNEG(rc);
		if ((snd_pcm_state(handle) == SND_PCM_STATE_SETUP) ||
				(snd_pcm_state(handle) == SND_PCM_STATE_PREPARED) )
		{
			/* fill some zeros to kick start*/
			rc = snd_pcm_writei(handle, zeroBuf, AUDIO_PERIOD_SIZE*(PLAY_MIN_PERIOD+1));
			if (rc < 0) {
				RETERRIFNEG(rc); 
				return -1;
			}
		}

		rc = snd_pcm_wait(handle, 2*AUDIO_PERIOD_MS);
		if (rc == 1)    /* data ready */
		{
			rc = snd_pcm_writei(handle, buffers, AUDIO_PERIOD_SIZE);
			if (rc < 0)
			{
				/* check if xrun and try recovery */
				if (rc == -EPIPE || rc == -ESTRPIPE)
				{
					rc = xrun_recovery(handle, rc);
					if (rc < 0)
					{
						RETERRIFNEG(rc);                            
					}
				}
				else
				{
					RETERRIFNEG(rc); 
					return -1;
				}
			}

		}
		else    /* 0:timeout, no data, < 0: error */
		{
			/* check if xrun and try recovery */
			if (rc == -EPIPE || rc == -ESTRPIPE)
			{
				rc = xrun_recovery(handle, rc);
				if (rc < 0)
				{
					RETERRIFNEG(rc);
				}
			}
		}
		RETERRIFNEG(rc);
	}
exit:
	printf("[audio_playback]The playback loop is exit!!\n");
	if (fps != NULL) fclose(fps);
	if (buffers!= NULL) free(buffers);
	if (zeroBuf!= NULL) free(zeroBuf);
	snd_pcm_drop(handle);
	snd_pcm_close(handle);	
	return 0;
}

static int  playback_func(snd_pcm_t *handle, unsigned int period_num_of_buffer, snd_pcm_uframes_t frame_num_of_period, int channel_number, unsigned int sample_size, FILE **fps, int mode)
{
	char **buffers = NULL;
	snd_timestamp_t tstamp;

	unsigned int frame_sec = 0;
	unsigned int frame_usec = 0;
	unsigned int pre_frame_sec = 0;
	unsigned int pre_freme_usec = 0;
	unsigned int read_periods =1;// period_num_of_buffer; 
	int rc  = 0, i = 0;
	unsigned int read_buffer_size = 0;
	unsigned int frame_size = sample_size * channel_number;
	unsigned int index = 0;

	/*NOTE: The size of audio buffer should be bigger than the size of snd_pcm_readi()/snd_pcm_readn() */
	read_buffer_size =  read_periods *1 *frame_num_of_period *frame_size; 

	printf("read_buffer_size = %d, frame_num_of_period = %d,  frame_size=%d, period_num_of_buffer=%d\n", read_buffer_size, frame_num_of_period, frame_size, period_num_of_buffer);
	buffers = (char **) malloc(channel_number * sizeof(char *));  
	memset(buffers, 0x0, channel_number * sizeof(char *));
	for (i = 0; i < channel_number; i++) {
		buffers[i] = (char *)malloc(read_buffer_size * sizeof(char));
		if (buffers[i] == NULL) {
			printf("[audio_playback]cannot allocate memory %u bytes\n", read_buffer_size);
			goto exit;
		}
	}

	printf("[audio_playback]write_buffer_size: %u, mode: %u\n", read_buffer_size, mode);

	//while (!feof(fps[0]))
	while (1)
	{
		if (bTerminate == 1) goto exit;

		rc = snd_pcm_writen(handle, (void *)buffers[0], read_buffer_size);

		printf("[audio_playback]write_buffer_size: %u, write_periods: %u\n", read_buffer_size, read_periods);
		if (rc == -EAGAIN) continue;
#if 1		
		if (rc < 0) {
			if (xrun_recovery(handle, rc) < 0) {
				printf("[audio_playback]error from read: %s\n",snd_strerror(rc));  
				goto exit;
				//continue;
			}
			continue;
		}
#endif
		if (rc != (int) read_buffer_size)
		{
			fprintf(stderr, "[audio_playback]short read, read %d frames(should be %d)\n", rc, (int) frame_num_of_period* read_periods);
		}

		if (bSleep) {
			unsigned int usecond = 1000 * 1000;
			printf("Sleep:%u\n", usecond);
			usleep(usecond);
		}
		if(fps[0] != NULL)
		{	
			rc = fread(buffers[0], sizeof(char), read_buffer_size, fps[0]);
			if (rc != read_buffer_size) fprintf(stderr, "[audio_playback]short write: wrote %d bytes\n", rc);
		}
#if 0	
		if ((mode == 0) && (fps != NULL)) {
			if (fps[0] != NULL) {
				rc = fread(buffers[0], sizeof(char), read_buffer_size, fps[0]);
				if (rc != read_buffer_size) fprintf(stderr, "[audio_playback]short write: wrote %d bytes\n", rc);
			}
		} else if ((mode == 1) && (fps != NULL)) {
			for (i = 0; i < channel_number; i++) {
				if (fps[i] != NULL) {
					rc = fread(buffers[i], sizeof(char), read_buffer_size, fps[i]);
					if (rc != read_buffer_size) fprintf(stderr, "[audio_playback]short write: wrote %d bytes\n", rc);
				}
			}
		}
#endif

	}
exit:
	printf("[audio_playback]The playback loop is exit!!\n");
	snd_pcm_drop(handle);
	snd_pcm_close(handle);

	if (buffers != NULL) {
		for (i = 0; i < channel_number; i++) {
			if (buffers[i] != NULL) {free((void *)(*(buffers+i))); buffers[i] = NULL;}
		}
		free(buffers);
		buffers = NULL;
	}

	return 0;
}

