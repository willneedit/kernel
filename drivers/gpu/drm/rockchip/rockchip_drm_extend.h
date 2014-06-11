#include "../../../video/rockchip/rk_drm_fb.h"
#include "../../../video/rockchip/hdmi/rk_hdmi.h"
#define WINDOWS_NR	4
#define MAX_HDMI_WIDTH   1920
#define MAX_HDMI_HEIGHT  1080

#define LAYER_NORMAL_WIN  	0
#define LAYER_VIDEO_WIN   	1
#define LAYER_CURSOR_WIN   	2
#define get_extend_context(dev)	platform_get_drvdata(to_platform_device(dev))
struct rk_overlay_api {
	unsigned int y_addr;
	unsigned int uv_addr;
	int format;

	int xpos;
	int ypos;
	int xact;
	int yact;
	int xsize;
	int ysize;

	int xvir;
};
struct extend_win_data {
	unsigned int offset_x;
	unsigned int offset_y;
	unsigned int ovl_width;
	unsigned int ovl_height;
	unsigned int fb_width;
	unsigned int fb_height;
	unsigned int bpp;
	unsigned int buf_offsize;
	unsigned int line_size;	/* bytes */
	dma_addr_t dma_addr;
	bool enabled;
	bool resume;
};

struct extend_context {
	unsigned int 		default_win;
	struct drm_crtc 	*crtc;
	struct rk_drm_display 	*drm_disp;
	struct extend_win_data 	win_data[WINDOWS_NR];
	struct mutex 		lock;
	struct rockchip_drm_subdrv subdrv;
	struct rockchip_drm_panel_info *panel;
	wait_queue_head_t 	wait_vsync_queue;
	atomic_t 		wait_vsync_event;
	int 			vblank_en;
	bool 			suspended;
};
