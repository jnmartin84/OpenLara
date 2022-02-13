#include <string.h>
#include <libdragon.h>

extern "C" {
#include <hfx.h>
#include <hfx_rb.h>
#include <hfx_cmds.h>
#include <hfx_int.h>
#include <hfx_types.h>
}

#include "game.h"

char str[256];
#define WND_TITLE    "OpenLara"

extern "C" {
extern void unlockVideo(display_context_t _dc);
extern display_context_t lockVideo(int i);
extern void *__safe_buffer[];
extern volatile unsigned long get_ticks_ms(void);
extern int __osGetFpcCsr();
extern int __osSetFpcCsr(int r);
}

// timing

#define USE_TIMER 1
#if USE_TIMER
volatile int timekeeping;
#endif

volatile int startTime=0;
display_context_t _dc;

#if USE_TIMER
void tickercb(int o) {
	timekeeping++;
}
#endif

volatile uint32_t last_time = 0;
volatile uint32_t cur_time = 0;
volatile uint32_t delta=0;
volatile uint32_t rollovers=0;

int called_times=0;

int osGetTimeMS() {
#if !USE_TIMER
	return get_ticks_ms() - startTime;
#endif
#if USE_TIMER
    return (timekeeping*17) - startTime;
#endif
}

    bool osSaveSettings()
    {
        return false;
    }

    bool osLoadSettings()
    {
        return false;
    }

    bool osCheckSave()
    {
        return false;
    }

    bool osSaveGame()
    {
        return false;
    }

    bool osLoadGame()
    {
        return false;
    }

    void osJoyVibrate(int32 index, int32 L, int32 R) {}

// sound
#define SND_FRAME_SIZE  4
#define SND_FRAMES      1024

// A Frame is a struct containing: int16 L, int16 R.
//Sound::Frame        *sndData;
//SDL_AudioDeviceID sdl_audiodev;

void sndFill(void *udata, uint8_t *stream, int len) {
        // Let's milk the audio subsystem for SND_FRAMES frames!
  //      Sound::fill(sndData, SND_FRAMES);
        // We have the number of samples, but each sample is 4 bytes long (16bit stereo sound),
        // and memcpy copies a number of bytes.
    //    memcpy (stream, sndData, SND_FRAMES * SND_FRAME_SIZE);
}


bool sndInit() {
#if 0 
 int FREQ = 44100;

    SDL_AudioSpec desired, obtained;

    desired.freq     = FREQ;
    desired.format   = AUDIO_S16SYS;
    desired.channels = 2;
    desired.samples  = SND_FRAMES;
    desired.callback = sndFill;
    desired.userdata = NULL;

    sdl_audiodev = SDL_OpenAudioDevice(NULL, 0, &desired, &obtained, /*SDL_AUDIO_ALLOW_FORMAT_CHANGE*/0);
    if (sdl_audiodev == 0)
    {
        LOG ("SDL2: error opening audio device: %s\n", SDL_GetError());
        return false;
    }

    if (desired.samples != obtained.samples) {
        LOG ("SDL2: number of samples not supported by device. Watch out for buggy audio drivers!\n");
        return false;
    }

    // Initialize audio buffer and fill it with zeros
    sndData = new Sound::Frame[SND_FRAMES];
    memset(sndData, 0, SND_FRAMES * SND_FRAME_SIZE);

    SDL_PauseAudioDevice(sdl_audiodev,0);

    return true;
#endif
	return false;
}

// multi-threading (no sound - no problem)
void* osMutexInit() { return NULL; }
void osMutexFree(void *obj) {}
void osMutexLock(void *obj) {}
void osMutexUnlock(void *obj) {}

void sndFree() {
//    SDL_PauseAudioDevice(sdl_audiodev,1);
  //  SDL_CloseAudioDevice(sdl_audiodev);

    // Delete the audio buffer
   // delete[] sndData;
}

// Input

#define MAX_JOYS 4
#define JOY_DEAD_ZONE_STICK      8192
#define WIN_W 640
#define WIN_H 480

bool fullscreen;

vec2 joyL, joyR;

bool osJoyReady(int index) {
    return index == 0; // TODO
}

void osJoyVibrate(int index, float L, float R) {
    // TODO
}

void toggleFullscreen () {

    uint32_t flags = 0;
    fullscreen = 1;
}


bool inputInit() {
controller_init();
    return true;
}

void inputFree() {
}

float joyAxisValue(int value) {
    if (value > -JOY_DEAD_ZONE_STICK && value < JOY_DEAD_ZONE_STICK)
        return 0.0f;
    return value / 32767.0f;
}

float joyTrigger(int value) {
    return min(1.0f, value / 255.0f);
}

vec2 joyDir(const vec2 &value) {
    float dist = min(1.0f, value.length());
    return value.normal() * dist;
}

void inputUpdate() {
	    controller_scan();

    struct controller_data keys_pressed = get_keys_down();
    struct controller_data keys_held = get_keys_held();
    struct controller_data keys_released = get_keys_up();
    struct SI_condat pressed = keys_pressed.c[0];

        if(pressed.left)     Input::setJoyDown(0,jkLeft, 1);
        if(pressed.right) Input::setJoyDown(0,jkRight, 1);
        if(pressed.up)         Input::setJoyDown(0,jkUp, 1);
        if(pressed.down) Input::setJoyDown(0,jkDown, 1);
        if(pressed.start) Input::setJoyDown(0,jkStart, 1);
		if(pressed.A) Input::setJoyDown(0,jkA,1);
		if(pressed.B) Input::setJoyDown(0,jkB,1);
		if(pressed.L) Input::setJoyDown(0,jkLB,1);
		if(pressed.R) Input::setJoyDown(0,jkRB,1);
		if(pressed.Z) Input::setJoyDown(0,jkSelect,1);
		if(pressed.C_down) Input::setJoyDown(0,jkX,1);
		if(pressed.C_left) Input::setJoyDown(0,jkY,1);

    struct SI_condat released = keys_released.c[0];
        if(released.left)     Input::setJoyDown(0,jkLeft, 0);
        if(released.right) Input::setJoyDown(0,jkRight, 0);
        if(released.up)         Input::setJoyDown(0,jkUp, 0);
        if(released.down) Input::setJoyDown(0,jkDown, 0);
        if(released.start) Input::setJoyDown(0,jkStart, 0);
		if(released.A) Input::setJoyDown(0,jkA,0);
		if(released.B) Input::setJoyDown(0,jkB,0);
		if(released.L) Input::setJoyDown(0,jkLB,0);
		if(released.R) Input::setJoyDown(0,jkRB,0);
		if(released.Z) Input::setJoyDown(0,jkSelect,0);
		if(released.C_down) Input::setJoyDown(0,jkX,0);
		if(released.C_left) Input::setJoyDown(0,jkY,0);
}

display_context_t lockVideo(int wait)
{
	display_context_t dc;

	if (wait)
	{
		while (!(dc = display_lock()));
	}
	else
	{
		dc = display_lock();
	}

	return dc;
}

void unlockVideo(display_context_t dc)
{
	if (dc)
	{
		display_show(dc);
	}
}

#define FPCSR_RM_RM 0x01000003

int main(int argc, char **argv) {
    int w, h;

    display_init(RESOLUTION_320x240, DEPTH_16_BPP, 2, GAMMA_NONE, ANTIALIAS_RESAMPLE);

#if USE_TIMER
    timer_init();
    timekeeping = 0;
    new_timer(781250, TF_CONTINUOUS, tickercb);
#endif

    register uint32 fpstat, fpstatset;
    fpstat = __osGetFpcCsr();
    // enable round to negative infinity for floating point
    fpstatset = (fpstat | FPCSR_RM_RM) ^ 2;
    // _Disable_ unimplemented operation exception for floating point.
    __osSetFpcCsr(fpstatset);

	int di = 0;

	di = dfs_init( DFS_DEFAULT_LOCATION );
	if(di != DFS_ESUCCESS)
	{
		printf("Could not initialize filesystem! %d\n", di);
		while(1);
	}

    Core::width  = 320;
    Core::height = 240;

    strcpy(contentDir, "/");
    strcpy(saveDir, contentDir);
    strcpy(cacheDir, contentDir);

    startTime =
#if USE_TIMER
	timekeeping*17 * 100000;
#endif
#if !USE_TIMER
	get_ticks_ms()*1048576;
#endif

    inputInit();

    char *lvlName =
	"DATA/LEVEL1.PHD";

    Game::init(lvlName);

	fpstat = __osGetFpcCsr();
	// enable round to negative infinity for floating point
	fpstatset = (fpstat | FPCSR_RM_RM) ^ 2;
	// _Disable_ unimplemented operation exception for floating point.
	__osSetFpcCsr(fpstatset);

	while (!Core::isQuit) {
        inputUpdate();

        if (Game::update())
	{
        Game::render();
        hfx_swap_buffers(GAPI::state);
	}
    };

    sndFree();
    Game::deinit();

    return 0;
}
