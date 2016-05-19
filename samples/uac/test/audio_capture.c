#include <signal.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/types.h>
#include <sys/stat.h>

#include <assert.h>
#include <fcntl.h>           /* For O_* constants */

#include <math.h>
#include "alsa/asoundlib.h"

#define DEFAULT_CHANNELS_NUM 2

#define MAX_SUPPORTED_CHANNEL_NUM 20

#define INTERLEAVED_MODE 0x0
#define NON_INTERLEAVED_MODE 0x1

//The device file of alsa could be checked through : /proc/asound/cards .
#define DEVFILE "hw:1,0"

static int bTerminate = 0;
static int bSleep = 0;

static snd_pcm_t *phandle = NULL;
static snd_pcm_t *handle = NULL;

static void sig_kill(int signo)
{
	printf("Receive SIGNAL: %s\n", strsignal(signo));
	if (bTerminate)
		return;

	if ((signo == SIGTERM) || (signo == SIGINT) || (signo == SIGABRT))
	{
		bTerminate = 1;

		if (phandle)
			snd_pcm_abort(phandle);

		if (handle)
			snd_pcm_abort(handle);

		if (signo == SIGABRT)
		{
			if (phandle)
			{
				snd_pcm_close(phandle);
				phandle = NULL;
			}

			if (handle)
			{
				snd_pcm_close(handle);
				handle = NULL;
			}

			/* Exit. */
			exit(EXIT_FAILURE);
		}
	}
	else if (signo == SIGHUP) {
		if (bSleep == 0)
			bSleep = 1;
		else
			bSleep =0;
		//bIssueEvent = 1;
	}

	signal(signo, SIG_DFL);
}
#define INTERLEAVED_FILE_NAME "interleaved.pcm"

#define RETERRIFNEG(x)	if(x<0) { printf("error at %s@%d: %s=%d\n", __func__, __LINE__, #x, x ); return x;}
//#define DEBUG
//#define TEST_MODE
#define SOURCE_TEST_MODE_SYS_PATH "/sys/class/plcm_usb/plcm0/f_audio_dual/audio_tmode"
#define AUDIO_SAMPLING_RATE	32000
#define PCM_NUM_CH			1
#define FIFO_NUM_CH			2

#define AUDIO_BUFFER_PERIOD     10
#define AUDIO_PERIOD_PER_SEC	100
#define AUDIO_PERIOD_MAX_SIZE   (AUDIO_SAMPLING_RATE/AUDIO_PERIOD_PER_SEC)
#define AUDIO_PERIOD_SIZE		(AUDIO_SAMPLING_RATE/AUDIO_PERIOD_PER_SEC)

#define PCM_FRAME_BYTES			(PCM_NUM_CH * 2)
#define FIFO_FRAME_BYTES		(FIFO_NUM_CH * 2)

#define PCM_PERIOD_BYTES	(AUDIO_PERIOD_SIZE * PCM_FRAME_BYTES)
#define FIFO_PERIOD_BYTES	(AUDIO_PERIOD_SIZE * FIFO_FRAME_BYTES)
#define AUDIO_BUFFER_SIZE	(AUDIO_PERIOD_SIZE * AUDIO_BUFFER_PERIOD)
#define PLAY_MIN_PERIOD         3
#define AUDIO_ACCESS_MODE       SND_PCM_ACCESS_RW_INTERLEAVED
#define AUDIO_PCM_FORMAT        SND_PCM_FORMAT_S16_LE


static unsigned int g_count = (unsigned int)-1;
static unsigned int g_is_splite = (unsigned int)-1;
static unsigned int g_bShowFrameRate = (unsigned int)-1;
static unsigned int g_offset= (unsigned int)-1;


static int do_splite(char *filename, unsigned int period_size, unsigned int channel_num, unsigned int sample_size, unsigned int period_count);
static int capture_func(snd_pcm_t *handle, unsigned int period_num_of_buffer, snd_pcm_uframes_t frame_num_of_period, int channel_number, unsigned int sample_size, FILE **fps, int mode);
static void gettimestamp(snd_pcm_t *handle, snd_timestamp_t *timestamp);

int  print_usage(void)
{
	printf(" -m : access mode,\n") ;
	printf("      this can be 0(interleaved), 1(non interleaved)\n");
	printf(" -n : channel number, this should be 1~%d\n", MAX_SUPPORTED_CHANNEL_NUM) ;
	printf(" -t : capture format, this can be : 0(SND_PCM_FORMAT_MU_LAW), 1(SND_PCM_FORMAT_A_LAW), 2(SND_PCM_FORMAT_S16_LE), 3(SND_PCM_FORMAT_S24_LE)\n") ;
	printf(" -d : capture device, this can be : 0:hw:0,0, 1:hw:0,1, 2:hw:1,0, 3:hw:1,1, 4:hw:2,0, 5:hw:2,1, 6:hw:3,0, 7:hw:3,1\n") ;
	printf(" -r : capture sample rate, this can be 8000, 16000, 32000, 44100, 48000.\n") ;
	printf(" -b : dump file or not\n") ;
	printf(" -c : capture count\n") ;
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

int main(int argc, char* argv[]) {
	int rc;

	int access_mode = INTERLEAVED_MODE ;

	int capture_format = 0 ;

	snd_pcm_uframes_t frame_num_of_period  = 0;
	unsigned int  period_num_of_buffer = 0; // number of periods per buffer

	int sample_size = 0 ;
	int frame_size   = 0;
	int period_size  = 0;
	int buffer_size  = 0;

	char *pcm_name;

	snd_pcm_hw_params_t *params;
	snd_pcm_format_t sample_format;

	unsigned int val = 0;
	int dir = 0;
	int channel_number = -1 ;

	int flag = 0;
#define SSM_NAME "ssm_test"
	FILE *fps[MAX_SUPPORTED_CHANNEL_NUM] = {NULL};

	unsigned int samplerate = 8000;

	char file_name[128] = {0x0};

	unsigned int dump_file = 0;

	int err = 0;

	{
		int i = 0;
		for (i = 0; i < MAX_SUPPORTED_CHANNEL_NUM; i++)
			fps[i] = NULL;
	}


	pcm_name =  strdup(DEVFILE);
	while ((flag=getopt(argc, argv, "n:m:t:d:r:b:c:s:S:o:h"))!=-1)
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
						capture_format =0;
						break;
					case '1':// SND_PCM_FORMAT_A_LAW
						capture_format = 1;
						break;
					case '2'://SND_PCM_FORMAT_S16_LE
						capture_format = 2;
						break;
					case '3'://SND_PCM_FORMAT_S24_LE (Signed 24 bit Little Endian using low three bytes in 32-bit word)
						capture_format = 3;
						break;
					default:
						capture_format = 0 ;
				}
				break;
			case 'd': // device
				/* Open PCM device for recording (capture). */
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
			case 'o':
				g_offset = (unsigned int)atoi(optarg);
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
	signal(SIGABRT, sig_kill);

	/* Open PCM device for playback. */
	rc = snd_pcm_open(&handle, pcm_name, SND_PCM_STREAM_CAPTURE, 0);
	if (rc < 0)
	{
		fprintf(stderr,	"[audio_capture] unable to open pcm device: %s, error:%d(%s)\n",
			pcm_name, rc, snd_strerror(rc));
		exit(1);
	}

	pcm_name = strdup("hw:2,0");
	rc = snd_pcm_open(&phandle, pcm_name, SND_PCM_STREAM_PLAYBACK, 0);
	if (rc < 0)
	{
		fprintf(stderr,	"[audio_play] unable to open pcm device: %s, error:%d(%s)\n",
			pcm_name, rc, snd_strerror(rc));
		exit(1);
	}

	rc = setHwParams(phandle, AUDIO_BUFFER_SIZE, AUDIO_PERIOD_SIZE, PCM_NUM_CH, AUDIO_SAMPLING_RATE);
	RETERRIFNEG(rc);
	rc = setSwParams(phandle, AUDIO_PERIOD_SIZE, AUDIO_BUFFER_SIZE*2, AUDIO_PERIOD_SIZE);
	RETERRIFNEG(rc);
	if(rc<0 && phandle)
	{
		snd_pcm_drop(phandle);
		snd_pcm_close(phandle);
	}

	//0 = block, 1 = nonblock mode, 2 = abort
	if (snd_pcm_nonblock(handle, 0) < 0) {
		fprintf(stderr,	"[audio_capture] unable to set  pcm device as block: %s\n", snd_strerror(rc));
		if (handle != NULL)
		{
			snd_pcm_drop(handle);
			snd_pcm_close(handle);
		}
		exit(1);
	}

	/* Allocate a hardware parameters object. */
	snd_pcm_hw_params_alloca(&params);

	/* Fill it in with default values. */
	snd_pcm_hw_params_any(handle, params);

	/* Set the desired hardware parameters. */
	if(access_mode == NON_INTERLEAVED_MODE)	{
		printf("[audio_capture] NON_INTERLEAVED_MODE\n");
		snd_pcm_hw_params_set_access(handle, params, SND_PCM_ACCESS_RW_NONINTERLEAVED);
	} else {
		printf("[audio_capture] INTERLEAVED_MODE\n");
		snd_pcm_hw_params_set_access(handle, params, SND_PCM_ACCESS_RW_INTERLEAVED);
	}
	/* Signed 16-bit little-endian format */
	if(capture_format == 0) {
		printf("[audio_capture] SND_PCM_FORMAT_MU_LAW !\n") ;
		snd_pcm_hw_params_set_format(handle, params, SND_PCM_FORMAT_MU_LAW);
		sample_format = SND_PCM_FORMAT_MU_LAW;
	} else if (capture_format == 1) {
		printf("[audio_capture] SND_PCM_FORMAT_A_LAW !\n") ;
		snd_pcm_hw_params_set_format(handle, params, SND_PCM_FORMAT_A_LAW);
		sample_format = SND_PCM_FORMAT_A_LAW;
	} else if (capture_format == 2)  {
		printf("[audio_capture] SND_PCM_FORMAT_S16_LE!\n") ;
		snd_pcm_hw_params_set_format(handle, params, SND_PCM_FORMAT_S16_LE);
		sample_format = SND_PCM_FORMAT_S16_LE;
	} else if (capture_format == 3)  {
		printf("[audio_capture] SND_PCM_FORMAT_U24_LE !\n") ;
		snd_pcm_hw_params_set_format(handle, params, SND_PCM_FORMAT_U24_LE);
		sample_format = SND_PCM_FORMAT_U24_LE;
	} else {
		printf("[audio_capture] SND_PCM_FORMAT_MU_LAW !\n") ;
		snd_pcm_hw_params_set_format(handle, params, SND_PCM_FORMAT_MU_LAW);
		sample_format = SND_PCM_FORMAT_MU_LAW;
	}

	sample_size = snd_pcm_format_physical_width(sample_format) / 8;
	printf("[audio_capture] sample_size  = %u bytes\n", (unsigned int)sample_size);

	/* Setting channel number */
	snd_pcm_hw_params_set_channels(handle, params, channel_number);
	printf("[audio_capture] channel number  = %u\n", (unsigned int)channel_number);

	/* Sampling rate (read from standard input parameters) */
	val = samplerate;
	dir = 0;
	if (snd_pcm_hw_params_set_rate_near(handle, params, &val, &dir) < 0)
	{   fprintf(stderr, "[audio_capture] Error setting periods.\n");
		exit(1);

	}
	printf("[audio_capture] sample rate  = %u HZ (near %u HZ)\n", (unsigned int)samplerate, (unsigned int)val);

	{
		dir = 0;
		unsigned int periods1 = 0;
		//snd_pcm_hw_params_get_periods_max(params, &period_num_of_buffer, &dir);
		period_num_of_buffer  = 2;
		snd_pcm_hw_params_set_periods(handle, params, period_num_of_buffer, dir);
		printf("[audio_capture] period per ring buffer = %u (periods)\n",  period_num_of_buffer);

		dir =0;

		snd_pcm_hw_params_get_periods( params, &periods1, &dir);

		printf("[audio_capture] period per ring buffer = %u (periods)\n",  periods1);
	}
#if 0
	dir = 0;
	snd_pcm_hw_params_get_period_size_max (params, &frame_num_of_period, &dir);
	printf("[audio_capture]max frames per period= %u (frames)\n", (unsigned int)frame_num_of_period);
#endif
	{
		unsigned int hoped_sample_rate = 16000;
		unsigned int hoped_frame_num_of_period = 1024;
		unsigned int ratio = samplerate/hoped_sample_rate;

		if (ratio == 0) ratio =1;

		frame_num_of_period = hoped_frame_num_of_period * ratio;
	}
	snd_pcm_hw_params_set_period_size_near(handle, params, &frame_num_of_period, dir);
	printf("[audio_capture]set frames per period= %u (frames)\n", (unsigned int)frame_num_of_period);
	{
		snd_pcm_uframes_t  frames = 0;
		dir = 0;
		snd_pcm_hw_params_get_period_size (params, &frames, &dir);
		printf("[audio_capture]get frames per period= %u (frames)\n", (unsigned int)frames);
	}
	/* Write the parameters to the driver */
	rc = snd_pcm_hw_params(handle, params);
	if (rc < 0)
	{
		fprintf(stderr, "[audio_capture] unable to set hw parameters: %s\n", snd_strerror(rc));
		exit(1);
	}

	{
		int err = 0;
		snd_pcm_sw_params_t*  swparams_c;
		snd_pcm_uframes_t buff_size = 0;
		snd_pcm_uframes_t boundary = 0;

		err = snd_pcm_hw_params_get_buffer_size(params, &buff_size);
		if (err < 0) {
			printf("[audio_capture]Unable to get buffer size for capture: %s\n", snd_strerror(err));
			exit(1);
		}
		printf("[audio_capture] ring buff size = %u (frames)\n", (unsigned int)buff_size);

		snd_pcm_sw_params_alloca(&swparams_c);
		/* get the current swparams */
		err = snd_pcm_sw_params_current(handle, swparams_c);
		if (err < 0) {
			printf("[audio_capture] Unable to determine current swparams_c: %s\n", snd_strerror(err));
			exit(1);
		}

		/* enable tstamp */
		err = snd_pcm_sw_params_set_tstamp_mode(handle, swparams_c, SND_PCM_TSTAMP_ENABLE);
		if (err < 0) {
			printf("[audio_capture] Unable to set tstamp mode : %s\n", snd_strerror(err));
			exit(1);
		}
	}

	if (dump_file == 0) goto process_capture;

	if (access_mode == NON_INTERLEAVED_MODE)
	{
		int i = 0;
		for (i = 0; i < channel_number; i++)
		{
			snprintf(file_name, sizeof(file_name),"%s-ch%d.pcm",  INTERLEAVED_FILE_NAME, i);
			fps[i] = fopen(file_name, "wb");
			if (fps[i] == NULL)
			{
				fprintf(stderr, "[audio_capture] unable to open file: %s\n", file_name);
				exit(1);
			}
			chmod(file_name, S_IRUSR|S_IRGRP|S_IROTH);
		}
	}
	else{
		strncpy(file_name,  INTERLEAVED_FILE_NAME, sizeof(file_name) );
		fps[0] = fopen(file_name, "wb");
		if (fps[0] == NULL)
		{
			fprintf(stderr, "[audio_capture] unable to open file: %s\n", file_name);
			exit(1);
		}
		chmod(file_name, S_IRUSR|S_IRGRP|S_IROTH);
	}
process_capture:
	if (access_mode == INTERLEAVED_MODE)
	{
		if (dump_file)
			capture_func(handle, period_num_of_buffer, frame_num_of_period, channel_number, sample_size, &fps[0], 0) ;
		else
			capture_func(handle, period_num_of_buffer, frame_num_of_period, channel_number, sample_size, NULL, 0) ;
	}
	else
	{
		if (dump_file)
			capture_func(handle, period_num_of_buffer, frame_num_of_period, channel_number, sample_size, &fps[0], 1) ;
		else
			capture_func(handle, period_num_of_buffer, frame_num_of_period, channel_number, sample_size, NULL, 1) ;
	}
exit:
	if (access_mode == NON_INTERLEAVED_MODE) {
		int i = 0;
		for (i = 0; i < channel_number; i++)
		{
			if (fps[i] != NULL) {
				fflush (fps[i]);
				fclose(fps[i]);
			}

		}
	} else {
		if (fps[0] != NULL) {
			fflush (fps[0]);
			fclose(fps[0]);
		}
	}
	//printf("[audio_capture](%d)\n", __LINE__);
	if (g_is_splite == 1) {
		if((access_mode == INTERLEAVED_MODE) && (g_count > 0))
			//	printf("[audio_capture](%d)\n", __LINE__);
			do_splite(INTERLEAVED_FILE_NAME, frame_num_of_period, channel_number, sample_size, g_count );
	}
	free(pcm_name);
	return 0;
}

static int xrun_recovery(snd_pcm_t *handle, int err) {

	if (err == -EPIPE) {    /* under-run */
		err = snd_pcm_prepare(handle);
		if (err < 0)
			printf("[audio_capture]Can't recovery from underrun, prepare failed: %s\n", snd_strerror(err));
		return 0;
	} else if (err == -ESTRPIPE) {
		while ((err = snd_pcm_resume(handle)) == -EAGAIN)
			sleep(1);       /* wait until the suspend flag is released */
		if (err < 0) {
			err = snd_pcm_prepare(handle);
			if (err < 0)
				printf("[audio_capture] Can't recovery from suspend, prepare failed: %s\n", snd_strerror(err));
		}
		return 0;
	}
	return err;
}

static int read_value_from_file(const char* filename, const char* format, int *value)
{
    int fd = 0;
    char buf[8];
    int len;

    fd = open(filename, O_RDONLY);

    if (fd < 0) {
        fprintf(stderr, "Can not open the file: %s, error: %d(%s)\n",
                filename, errno, strerror(errno));
        return -1;
    }

    len = read(fd, buf, sizeof(buf));

    if (len <= 0) {
        fprintf(stderr, "Can not read data from file: %s, error: %d(%s)\n",
                filename, errno, strerror(errno));
        close(fd);
        return -1;
    }

    len = sscanf(buf, format, value);

    if (len <= 0) {
        fprintf(stderr, "Can not parse the value from %s\n", filename);
        close(fd);
        return -1;
    }

    close(fd);
    return 0;
}

static int capture_func(snd_pcm_t *handle, unsigned int period_num_of_buffer, snd_pcm_uframes_t frame_num_of_period, int channel_number, unsigned int sample_size, FILE **fps, int mode)
{
	char **buffers = NULL;
	snd_timestamp_t tstamp;

	unsigned int frame_sec = 0;
	unsigned int frame_usec = 0;
	unsigned int pre_frame_sec = 0;
	unsigned int pre_freme_usec = 0;
	unsigned int read_periods =1;// period_num_of_buffer;
	int rc  = 0, i = 0, j=0;
	unsigned int read_buffer_size = 0;
	unsigned int frame_size = sample_size * channel_number;
	unsigned int index = 0;
	char *wbuffers = NULL;
	int size = 0;
	struct timeval tps, tpsend;

	/*NOTE: The size of audio buffer should be bigger than the size of snd_pcm_readi()/snd_pcm_readn() */
	read_buffer_size =  read_periods *1 *frame_num_of_period *frame_size;
	size = read_buffer_size/channel_number;
	wbuffers = (char *) malloc(size);
	memset(wbuffers, 0x0, size);

	printf("read_buffer_size = %d, frame_num_of_period = %d,  frame_size=%d, period_num_of_buffer=%d\n", read_buffer_size, frame_num_of_period, frame_size, period_num_of_buffer);
	buffers = (char **) malloc(channel_number * sizeof(char *));
	memset(buffers, 0x0, channel_number * sizeof(char *));
	for (i = 0; i < channel_number; i++) {
		buffers[i] = (char *)malloc(read_buffer_size * sizeof(char));
		if (buffers[i] == NULL) {
			printf("[audio_capture]cannot allocate memory %u bytes\n", read_buffer_size);
			goto exit;
		}
	}

	printf("[audio_capture]read_buffer_size: %u, mode: %u\n", read_buffer_size, mode);

	while (1)
	{
		if (bTerminate == 1) goto exit;

#ifdef DEBUG
		clock_gettime(CLOCK_REALTIME, &tps);
#endif
		if (mode == 0)
			rc = snd_pcm_readi(handle, (void *)buffers[0], frame_num_of_period *  read_periods);
		else
			rc = snd_pcm_readi(handle, (void **)buffers, frame_num_of_period *  read_periods);

		if (rc == -EAGAIN) continue;
#if 1
		if (rc < 0) {
			if (xrun_recovery(handle, rc) < 0) {
				printf("[audio_capture]error from read: %s\n",snd_strerror(rc));
				goto exit;
				//continue;
			}
			continue;
		}
#endif
		if (rc != (int) frame_num_of_period* read_periods)
		{
			fprintf(stderr, "[audio_capture]short read, read %d frames(should be %d)\n", rc, (int) frame_num_of_period* read_periods);
		}

		if (bSleep) {
			unsigned int usecond = 1000 * 1000;
			printf("Sleep:%u\n", usecond);
			usleep(usecond);
		}
		if (bTerminate == 1)
			break;
		if ((mode == 0) && (fps != NULL)) {
			if (fps[0] != NULL) {
				memset(wbuffers, 0x0, size);
#ifdef TEST_MODE
				#define TWO_PI (3.1415926 * 2)
				int sampleRate = 32000;
				float rate = 1;
				short amplitude = 128;
				float frequency = 1000;
				int n=0;
				int sample=0;
				int p_mode;

				read_value_from_file(SOURCE_TEST_MODE_SYS_PATH, "%d\n", &p_mode);
				if(p_mode > 0)
				{
					/*Here's some example code that makes a 1 kHz sample at a 32 kHz sample rate and with 16 bit samples (that is, not floating point)*/
					for (i = 0, j = 0, n=0; i < read_buffer_size; n++){
						sample = rate*amplitude * sin((TWO_PI * n * frequency) / sampleRate);
						wbuffers[j+1] = (char)(sample&0xff);
						wbuffers[j]= (char)(sample>>8&0xff);
						i+=frame_size;
						j+=sample_size;
					}
				} else {
					for (i = 0,j = 0; i < read_buffer_size;){
						wbuffers[j+1] = buffers[0][i+g_offset];
						wbuffers[j]= buffers[0][i +1+g_offset];
						i+=frame_size;
						j+=sample_size;
					}
				}
#else
				for (i = 0,j = 0; i < read_buffer_size;){
					wbuffers[j+1] = buffers[0][i+g_offset];
					wbuffers[j]= buffers[0][i +1+g_offset];
					i+=frame_size;
					j+=sample_size;
				}
#endif

#ifdef DEBUG
				clock_gettime(CLOCK_REALTIME, &tpsend);
				printf("[audio_capture]wbuffers = %d wsize =%d frame_size %d read_buffer_size %d chn %d %lu s %lu ms\n",j,size,frame_size,read_buffer_size, g_offset, tpsend.tv_sec-tps.tv_sec, (tpsend.tv_usec-tps.tv_usec)/1000000);				clock_gettime(CLOCK_REALTIME, &tps);

				rc = fwrite(wbuffers, sizeof(char), size, fps[0]);
				if (rc != size) fprintf(stderr, "[audio_capture]short write: wrote %d bytes\n", rc);
#endif

#if 1
				if (rc)    /* data ready */
				{
					rc = snd_pcm_writei(phandle, wbuffers, frame_num_of_period);
					if (rc < 0)
					{
						/* check if xrun and try recovery */
						if (rc == -EPIPE || rc == -ESTRPIPE)
						{
							rc = xrun_recovery(phandle, rc);
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
						rc = xrun_recovery(phandle, rc);
						if (rc < 0)
						{
							RETERRIFNEG(rc);
						}
					}
				}
#endif
#ifdef DEBUG
				clock_gettime(CLOCK_REALTIME, &tpsend);
				printf("[audio_write]wbuffers = %d frame_size %d read_buffer_size %d chn %d %lu s %lu ms\n",j,frame_size,read_buffer_size, g_offset, tpsend.tv_sec-tps.tv_sec, (tpsend.tv_usec-tps.tv_usec)/1000000);
#endif
			}
		} else if ((mode == 1) && (fps != NULL)) {
			for (i = 0; i < channel_number; i++) {
				if (fps[i] != NULL) {
					rc = fwrite(buffers[i], sizeof(char), read_buffer_size, fps[i]);
					if (rc != read_buffer_size) fprintf(stderr, "[audio_capture]short write: wrote %d bytes\n", rc);
				}
			}
		}

		if (g_bShowFrameRate == 1) {
			unsigned int tmp_frame_sec = 0;
			unsigned int tmp_frame_usec = 0;
			signed int  diff  = 0;
			gettimestamp(handle, &tstamp);
			frame_sec  = (unsigned int)(tstamp.tv_sec);
			frame_usec = (unsigned int)(tstamp.tv_usec );
			tmp_frame_sec = frame_sec;
			tmp_frame_usec = frame_usec ;
			diff  = ((signed int)tmp_frame_sec  - (signed int)pre_frame_sec) * 1000000+ ((signed int)tmp_frame_usec  - (signed int)pre_freme_usec);
			printf("[audio_capture]frame interval = %f (msec) (%u)\n",(float)diff/1000.0, index);
			pre_frame_sec = tmp_frame_sec ;
			pre_freme_usec = tmp_frame_usec;
		}

#if 0
		if (g_count != (unsigned int)-1) {

			if ((++index) == g_count) {
				printf("[audio_capture] Because got : %u, now exit!! \n", g_count );
				goto exit;
			}
		}
#endif


	}
exit:
	printf("[audio_capture]The capture loop is exit!!\n");
	snd_pcm_drop(handle);
	snd_pcm_close(handle);
	snd_pcm_drop(phandle);
	snd_pcm_close(phandle);
	if (wbuffers!= NULL) free(wbuffers);

	if (buffers != NULL) {
		for (i = 0; i < channel_number; i++) {
			if (buffers[i] != NULL) {free((void *)(*(buffers+i))); buffers[i] = NULL;}
		}
		free(buffers);
		buffers = NULL;
	}

	return 0;
}


static void gettimestamp(snd_pcm_t *handle, snd_timestamp_t *timestamp)
{
	int i_snd_rc = -1;

	snd_pcm_status_t *p_status;


	/* Get the status */
	snd_pcm_status_alloca(&p_status);
	i_snd_rc = snd_pcm_status(handle, p_status );
	if( i_snd_rc < 0 )
	{
		printf("[audio_capture]Stream status error: %s\n", snd_strerror(i_snd_rc));
		return;
	}
	/* Handle buffer underruns and get the status again */
	if( snd_pcm_status_get_state( p_status ) == SND_PCM_STATE_XRUN )
	{
		/* Prepare the device */
		i_snd_rc = snd_pcm_prepare( handle );
		if( i_snd_rc )
		{
			printf("[audio_capture]cannot recover from buffer underrun\n");
			return;
		}
		printf("[audio_capture]recovered from buffer underrun\n" );
		/* Get the new status */
		i_snd_rc = snd_pcm_status( handle, p_status );
		if( i_snd_rc < 0 )
		{
			printf("[audio_capture]cannot get device status after recovery: %s\n", snd_strerror(i_snd_rc) );
			return;
		}
	}
	else
	{
		snd_pcm_status_get_tstamp(p_status, timestamp);
	}
	return;
}
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
#define MAX_SUPPORTED_CHANNEL_NUM  20
unsigned char *channel_pcm_buffers[MAX_SUPPORTED_CHANNEL_NUM] = {0};

static int allocate_channel_pcm_buffers(unsigned int channel_num, unsigned int channel_buffer_size);
static int free_channel_pcm_buffers(unsigned int channel_num);
static int extract_channel_pcm(unsigned int channel_num, unsigned int period_size, unsigned int period_count, unsigned int sample_size, unsigned char *pcm_buffer);


static int do_splite(char *filename, unsigned int period_size, unsigned int channel_num, unsigned int sample_size, unsigned int period_count)  {


	unsigned int expected_file_size = 0;
	unsigned char *pcm_buffer =  NULL;
	FILE *pFile = NULL;
	unsigned int lSize = 0;
	size_t result = 0;
	unsigned int frame_size = 0;

	frame_size = (unsigned int)(channel_num * sample_size);
	expected_file_size = (unsigned int)(period_size * frame_size *period_count);

	if(channel_num > MAX_SUPPORTED_CHANNEL_NUM) goto exit;

	pFile = fopen ( filename , "rb" );
	if (pFile==NULL) {fputs ("File error",stderr); exit (1);}

	// obtain file size:
	fseek (pFile , 0 , SEEK_END);
	lSize = ftell (pFile);
	rewind (pFile);

	if (lSize != expected_file_size) {
		//printf("file size %u != %u\n", lSize, expected_file_size);
		goto exit;
	}

	pcm_buffer = (unsigned char *)malloc(expected_file_size);
	if (!pcm_buffer) goto exit;
#if 1
	printf("period_size : %d\n", period_size);
	printf("channel_num : %d\n", channel_num);
	printf("sample_size  : %d\n", sample_size);
	printf("period_count : %d\n", period_count);
	printf("frame_size : %d\n", frame_size);
	printf("expected_file_size : %u\n", expected_file_size);
#endif
	result = fread (pcm_buffer,1, lSize,pFile);
	if (result != lSize) {fputs ("Reading error",stderr); exit (3);}

#if 0
	{
		unsigned int i = 0;
		for (i = 0; i <  frame_size; i++){
			if ((i > 0) && (i == (frame_size >> 1))) printf("\n");
			printf("%02x, ",  pcm_buffer[i]);
		}
		printf("\n");
	}
#endif

	if (allocate_channel_pcm_buffers(channel_num, period_size * 1  *sample_size  *period_count)!=0) goto exit;

	extract_channel_pcm(channel_num, period_size, period_count, sample_size, pcm_buffer);

exit:
	if (pFile != NULL) fclose(pFile);
	if (pcm_buffer != NULL) free(pcm_buffer);
	free_channel_pcm_buffers(channel_num);
	return 0;
}

int allocate_channel_pcm_buffers(unsigned int channel_num, unsigned int channel_buffer_size){
	int i = 0;

	if (channel_num > MAX_SUPPORTED_CHANNEL_NUM) return -1;

	for(i = 0; i < channel_num; i++){
		channel_pcm_buffers[i] = NULL;
		channel_pcm_buffers[i] = (unsigned char*)malloc(channel_buffer_size);
		if (channel_pcm_buffers[i] == NULL) return -1;
		memset(channel_pcm_buffers[i] , 0x0, channel_buffer_size);
	}

	return 0;
}


int free_channel_pcm_buffers(unsigned int channel_num){
	int i = 0;

	if (channel_num > MAX_SUPPORTED_CHANNEL_NUM) return -1;


	for(i = 0; i < channel_num; i++){
		if (channel_pcm_buffers[i] != NULL) free(channel_pcm_buffers[i]);
	}

	return 0;
}

int extract_channel_pcm(unsigned int channel_num, unsigned int period_size, unsigned int period_count, unsigned int sample_size, unsigned char *pcm_buffer){

	int i = 0;
	int j = 0;
	int k = 0;
	int l = 0;
	int s = 0;
	unsigned short *src_16 = NULL;
	unsigned short *dst_16 =NULL;
	unsigned char *src = NULL;
	unsigned  char *dst =NULL;
	unsigned int frame_count = period_count *period_size;
	unsigned int frame_size = sample_size * channel_num;
	unsigned int offset = 0;
	unsigned char *buffer = pcm_buffer;
	unsigned int one_channel_size = period_size * 1  *sample_size  *period_count;
	unsigned int first = 1;
	FILE *fps[MAX_SUPPORTED_CHANNEL_NUM] = {0};
	char ch_file_name[128];

	if (pcm_buffer == NULL) return -1;

	if (channel_num > MAX_SUPPORTED_CHANNEL_NUM) return -1;

	for (i = 0; i < channel_num; i++) {
		memset(ch_file_name, 0x0, sizeof(ch_file_name));
		snprintf (ch_file_name, sizeof(ch_file_name), "ch%d.pcm", i );
		fps[i] = fopen(ch_file_name, "wb");
	}

	//period_size * 1  *sample_size  *period_count
	printf("sample_size=%u\n", sample_size);
	for (i = 0; i < frame_count; i++ ){

		for (j = 0; j <  channel_num; j++) {
			channel_pcm_buffers[j][offset] = buffer[k];
			channel_pcm_buffers[j][offset + 1]= buffer[k + 1];
			k += sample_size;
		}
		if (first ==1)
		{
			first = 0;
			printf("k=%d channel_num = %d\n", k, channel_num);
		}
		k = 0;
		offset +=  sample_size;
		buffer+= frame_size;
	}

	for (i = 0; i < channel_num; i++) {
		printf("[%d]:", i);
		for (j = 0; j < 16; j++) {
			printf("%02x, ", channel_pcm_buffers[i][j] );
		}
		printf("\n");
		if (fps[i] != NULL) {
			fwrite (channel_pcm_buffers[i] , sizeof(char),one_channel_size, fps[i]);
			fclose(fps[i]);
		}
	}
	return 0;
}
