// SPDX-License-Identifier: GPL-3.0
// For this driver /drivers/gpu/drm/tiny/gm12u320.c by
// Hans de Goede <hdegoede@redhat.com> has been used as
// a blueprint.

#include <linux/module.h>
#include <linux/usb.h>
#include <drm/drm_atomic_helper.h>
#include <drm/drm_atomic_state_helper.h>
#include <drm/drm_connector.h>
#include <drm/drm_damage_helper.h>
#include <drm/drm_drv.h>
#include <drm/drm_edid.h>
#include <drm/drm_fb_helper.h>
#include <drm/drm_fb_dma_helper.h>
#include <drm/drm_file.h>
#include <drm/drm_format_helper.h>
#include <drm/drm_fourcc.h>
#include <drm/drm_framebuffer.h>
#include <drm/drm_gem_atomic_helper.h>
#include <drm/drm_gem_framebuffer_helper.h>
#include <drm/drm_gem_shmem_helper.h>
#include <drm/drm_gem_dma_helper.h>
#include <drm/drm_ioctl.h>
#include <drm/drm_managed.h>
#include <drm/drm_modeset_helper_vtables.h>
#include <drm/drm_probe_helper.h>
#include <drm/drm_simple_kms_helper.h>
#include <drm/drm_gem_shmem_helper.h>

#define DRIVER_NAME "usb2epdiy"
#define DRIVER_DESC "USB e-paper display with EPDiy-controller"
#define DRIVER_DATE "2024"
#define DRIVER_MAJOR 1
#define DRIVER_MINOR 0

#define WHITE_THRESHOLD 200

#define DATA_TIMEOUT		msecs_to_jiffies(1500)
#define DATA_SND_EPT		0x02
#define DATA_RCV_EPT		0x82
#define USB_RCV_BUF_SIZE	64
#define USB_SND_BUF_SIZE	(128*64)  // number_of_chunks * chunk_size at esp

#define usb2epdiy_ERR(fmt, ...) DRM_DEV_ERROR(usb2epdiy->drmdev.dev, fmt, ##__VA_ARGS__)

#define to_usb2epdiy(__drmdev) container_of(__drmdev, struct usb2epdiy_device, drmdev)

enum command {
    COMMAND_FOLLOWS = 128,
    SEND_128_BYTE   = 0,
    UNKNOWN_COMMAND = 254,

    SCREEN_SIZE     = 10,

    CLEAR_DISPLAY   = 100,
    REFRESH_DISPLAY = 101,
    DRAW_IMAGE_2BIT = 150
};

struct usb2epdiy_device {
	struct drm_device drmdev;
	struct device *dmadev;
	struct usb_device *usbdev;
	struct drm_simple_display_pipe pipe;
	struct drm_connector conn;
	struct mutex fb_lock;
	bool fb_is_dirty;
	u8 *fb_buf;
	u8 *fb_buf_outgoing;
	u8 *fb_buf_previous;
	uint16_t width;
	uint16_t height;
	bool pipe_enabled;
	struct delayed_work fb_update_work;
	unsigned char *usb_rcv_buf;
	unsigned char *usb_snd_buf;
	size_t usb_snd_buf_filled;
	struct delayed_work digest_stream_work;
	u8 last_refresh_id;
	u8 last_refresh_id_rcv;
};

static void serial_flush(
	struct usb2epdiy_device *usb2epdiy
) {
	int actual_len;
	int ret = 0;
	ret = usb_bulk_msg(usb2epdiy->usbdev,
		usb_sndbulkpipe(usb2epdiy->usbdev, DATA_SND_EPT),
		usb2epdiy->usb_snd_buf, usb2epdiy->usb_snd_buf_filled,
		&actual_len, DATA_TIMEOUT);
	if(usb2epdiy->usb_snd_buf_filled == actual_len) {
			usb2epdiy->usb_snd_buf_filled = 0;
	} else {
		memcpy(usb2epdiy->usb_snd_buf, usb2epdiy->usb_snd_buf+actual_len, usb2epdiy->usb_snd_buf_filled-actual_len);
		usb2epdiy->usb_snd_buf_filled -= actual_len;
	}
	if (ret) {
		usb2epdiy_ERR("usb_bulk_msg error: %d\n", ret);
	}
}

static void write_serial(
	struct usb2epdiy_device *usb2epdiy,
	unsigned char *bytes,
	size_t len
) {
	if(len==0)
		return;
	size_t copy_size = 0;
	while(len) {
		copy_size = min(USB_SND_BUF_SIZE - usb2epdiy->usb_snd_buf_filled, len);
		memcpy(usb2epdiy->usb_snd_buf+usb2epdiy->usb_snd_buf_filled, bytes, copy_size);
		usb2epdiy->usb_snd_buf_filled += copy_size;
		bytes += copy_size;
		len -= copy_size;
		if(usb2epdiy->usb_snd_buf_filled >= USB_SND_BUF_SIZE) {
			serial_flush(usb2epdiy);
		}
	}
}

static void write_payload_bytes(
	struct usb2epdiy_device *usb2epdiy,
	unsigned char *bytes,
	unsigned int len
) {
    unsigned char cmd128[] = {COMMAND_FOLLOWS, SEND_128_BYTE};
	unsigned int copy_start = 0;
    for(unsigned int i=0; i<len; i++) {
        if( bytes[i]==0x80 ) {
			write_serial(usb2epdiy, &bytes[copy_start], i-copy_start);
            write_serial(usb2epdiy, cmd128, sizeof(cmd128));
			copy_start = i+1;
        }
    }
	write_serial(usb2epdiy, &bytes[copy_start], len-copy_start);
}

static void write_command(
	struct usb2epdiy_device *usb2epdiy,
	unsigned char cmdno,
	unsigned char *payload,
	unsigned int plen
) {
    unsigned char cmd[] = {
        COMMAND_FOLLOWS,
        0x00
    };
    cmd[1] = cmdno;
    write_serial(usb2epdiy, cmd, 2);
    if( payload )
        write_payload_bytes(usb2epdiy, payload, plen);
}

static void epd_refresh_display(
	struct usb2epdiy_device *usb2epdiy
) {
	usb2epdiy->last_refresh_id++;
	write_command(usb2epdiy, REFRESH_DISPLAY, &(usb2epdiy->last_refresh_id), 1);
}

static void epd_send_image_header(
	struct usb2epdiy_device *usb2epdiy,
	uint16_t x, uint16_t y,
	uint16_t width, uint16_t height
) {
	// the esp32-s3 is little endian
	unsigned char sizeinfo[] = {
		(x) & 0xFF,
		(x>>8) & 0xFF,
		(y) & 0xFF,
		(y>>8) & 0xFF,
		(width) & 0xFF,
		(width>>8) & 0xFF,
		(height) & 0xFF,
		(height>>8) & 0xFF
	};
	write_command(usb2epdiy, DRAW_IMAGE_2BIT, sizeinfo, sizeof(sizeinfo));  // Send image header
	// After this, the caller needs to send the pixel data themselves!
}

#define rep_2bit_stage1 ((1<<4)-1)
#define rep_2bit_stage2 ((1<<11)-1)
#define rep_2bit_stage3 ((1<<18)-1)
#define rep_2bit_stage4 ((1<<25)-1)
static void epd_send_image_pixels_repeats_2bit(
	struct usb2epdiy_device *usb2epdiy,
	u_int8_t color, int64_t repeats
) {
    if( repeats <= rep_2bit_stage1 ) {
        u_int8_t tosend = ((repeats<<3)&(rep_2bit_stage1<<3)) | (color & 0b111);
        write_payload_bytes(usb2epdiy, &tosend, 1);
    } else if( repeats <= rep_2bit_stage2 ) {
        u_int8_t tosend[2];
        tosend[0] = 0x80 | ((repeats>>4)&(rep_2bit_stage1<<3)) | (color & 0b111);
        tosend[1] = repeats&0x7F;
        write_payload_bytes(usb2epdiy, tosend, 2);
    } else if( repeats <= rep_2bit_stage3 ){
        u_int8_t tosend[3];
        tosend[0] = 0x80 | ((repeats>>11)&(rep_2bit_stage1<<3)) | (color & 0b111);
        tosend[1] = 0x80 | ((repeats>>7)&0x7F);
        tosend[2] = repeats&0x7F;
        write_payload_bytes(usb2epdiy, tosend, 3);
    } else if( repeats <= rep_2bit_stage4 ){
        u_int8_t tosend[4];
        tosend[0] = 0x80 | ((repeats>>18)&(rep_2bit_stage1<<3)) | (color & 0b111);
        tosend[1] = 0x80 | ((repeats>>14)&0x7F);
        tosend[2] = 0x80 | ((repeats>>7)&0x7F);
        tosend[3] = repeats&0x7F;
        write_payload_bytes(usb2epdiy, tosend, 4);
    } else {
        while (repeats>rep_2bit_stage4)
        {
            epd_send_image_pixels_repeats_2bit(usb2epdiy, color, rep_2bit_stage4);
            repeats-=(rep_2bit_stage4+1);
        }
        epd_send_image_pixels_repeats_2bit(usb2epdiy, color, repeats);
    }
}

static void find_damage(int width, int height, int strides, u8* a, u8* b, struct drm_rect *rect) {
	int x1=width, y1=height, x2=-1, y2=-1;
	u8 *a_row, *b_row;
	for( int y=0; y<height; y++) {
		a_row = a + y*strides;
		b_row = b + y*strides;
		int x;
		for( x=0; x<width; x++) {
			if(a_row[x] != b_row[x]) {
				if(x1>x) x1=x;
				if(y1>y) y1=y;
				if(x2<x) x2=x;
				if(y2<y) y2=y;
				break;
			}
		}
		int stop = x;
		if(y2==y) {
			// If we already know the damage rectangle extends down to this line as well, we only have
			// to check the right side of the rectangle.
			stop = max(stop, x2);
		}
		for( x=width-1; x>stop; x--) {
			if(a_row[x] != b_row[x]) {
				if(x1>x) x1=x;
				if(y1>y) y1=y;
				if(x2<x) x2=x;
				if(y2<y) y2=y;
				break;
			}
		}
	}
	if((x1>x2) || (y1>y2)) {
		x1=0; y1=0; x2=0; y2=0;
	}

	printk(KERN_INFO "Damage: %d,%d - %d,%d\n", x1, y1, x2-x1+1, y2-y1+1);

	rect->x1 = x1;
	rect->y1 = y1;
	rect->x2 = x2;
	rect->y2 = y2;
}

bool epdiy_is_ready(struct usb2epdiy_device *usb2epdiy) {
	return usb2epdiy->width && (usb2epdiy->last_refresh_id == usb2epdiy->last_refresh_id_rcv);
}

#define MODE_COLOR          0b0001
#define MODE_TRANSPARENT    0b0010
#define MODE_INVERT         0b0100

static void usb2epdiy_send_fb(struct usb2epdiy_device *usb2epdiy)
{
	uint16_t x1=0, y1=0, x2=usb2epdiy->width, y2=usb2epdiy->height;
	u8 *buf_row;
	u8 *buf_row_previous;
	struct drm_rect rect;

	if(!usb2epdiy->fb_is_dirty)
		return;

	if(!epdiy_is_ready(usb2epdiy))
		return;

	mutex_lock(&usb2epdiy->fb_lock);
	swap(usb2epdiy->fb_buf, usb2epdiy->fb_buf_outgoing);
	usb2epdiy->fb_is_dirty = false;
	mutex_unlock(&usb2epdiy->fb_lock);

	printk(KERN_INFO "Sending framebuffer to display\n");

	find_damage(
		usb2epdiy->width, usb2epdiy->height, usb2epdiy->width,
		usb2epdiy->fb_buf_outgoing, usb2epdiy->fb_buf_previous,
		&rect
	);
	x1 = rect.x1;
	y1 = rect.y1;
	x2 = rect.x2+1;
	y2 = rect.y2+1;

	epd_send_image_header(usb2epdiy, x1, y1, x2-x1, y2-y1);
	u_int8_t color = 0;
	bool img_has_fb_color = false;
	bool img_has_inv_fb_color = false;
	int64_t repeats = -1;
	u_int8_t mode = 0;
	u8 thiscolor;
	u8 previous_color;
	for( u_int16_t y=y1; y<y2; y++) {
		buf_row = usb2epdiy->fb_buf_outgoing + y*usb2epdiy->width;
		buf_row_previous = usb2epdiy->fb_buf_previous + y*usb2epdiy->width;
		for( u_int16_t x=x1; x<x2; x++) {

			thiscolor = ((buf_row[x] >> 4)*(buf_row[x] >> 4))>>6;
			previous_color = ((buf_row_previous[x] >> 4)*(buf_row_previous[x] >> 4))>>6;
			

			img_has_fb_color = thiscolor == previous_color;
			img_has_inv_fb_color = thiscolor == (1-previous_color);

			if( repeats == -1 ) {
				color = thiscolor;
				repeats = 0;
				mode = MODE_COLOR;
				if(img_has_fb_color)
					mode |= MODE_TRANSPARENT;
				if(img_has_inv_fb_color)
					mode |= MODE_INVERT;
			} else {
				if( (mode & MODE_TRANSPARENT) && img_has_fb_color ) {
					repeats += 1;
					mode &= ~MODE_INVERT;
					if(color != thiscolor)
						mode &= ~MODE_COLOR;
				} else if( (mode & MODE_INVERT) && img_has_inv_fb_color ) {
					repeats += 1;
					mode &= ~MODE_TRANSPARENT;
					if(color != thiscolor)
						mode &= ~MODE_COLOR;
				} else if((mode & MODE_COLOR) && (color == thiscolor) ) {
					repeats += 1;
					mode  &= ~(MODE_TRANSPARENT|MODE_INVERT);
				} else {

					if(mode & MODE_COLOR) {
						epd_send_image_pixels_repeats_2bit(usb2epdiy, color&11, repeats);
					} else if(mode & MODE_TRANSPARENT) {
						epd_send_image_pixels_repeats_2bit(usb2epdiy, 0b100, repeats);
					} else if(mode & MODE_INVERT) {
						epd_send_image_pixels_repeats_2bit(usb2epdiy, 0b101, repeats);
					}
					
					color = thiscolor;
					repeats = 0;
					mode = MODE_COLOR;
					if(img_has_fb_color)
						mode |= MODE_TRANSPARENT;
					if(img_has_inv_fb_color)
						mode |= MODE_INVERT;
				}
			}
		}
	}

	if(mode & MODE_COLOR) {
		epd_send_image_pixels_repeats_2bit(usb2epdiy, color&11, repeats);
	} else if(mode & MODE_TRANSPARENT) {
		epd_send_image_pixels_repeats_2bit(usb2epdiy, 0b100, repeats);
	} else if(mode & MODE_INVERT) {
		epd_send_image_pixels_repeats_2bit(usb2epdiy, 0b101, repeats);
	}

	//epd_refresh_area(usb2epdiy, x1, y1, x2-x1, y2-y1);
	epd_refresh_display(usb2epdiy);
	serial_flush(usb2epdiy);
	memcpy(usb2epdiy->fb_buf_previous, usb2epdiy->fb_buf_outgoing, (size_t)usb2epdiy->width * usb2epdiy->height);
}


static void usb2epdiy_send_fb_worker(struct work_struct *work) {
	struct usb2epdiy_device *usb2epdiy = container_of(
		to_delayed_work(work),
		struct usb2epdiy_device,
		fb_update_work
	);
	if(!usb2epdiy->pipe_enabled)
		return;
	usb2epdiy_send_fb(usb2epdiy);
}



/* ------------------------------------------------------------------ */
/* usb2epdiy connector						      */

static int usb2epdiy_conn_get_modes(struct drm_connector *connector)
{
	static struct drm_display_mode usb2epdiy_mode;
	struct usb2epdiy_device *usb2epdiy = to_usb2epdiy(connector->dev);
	usb2epdiy_mode = (struct drm_display_mode){
		DRM_MODE_INIT(
			20, // refresh rate in hertz
			usb2epdiy->width,
			usb2epdiy->height,
			271, // width in mm
			203	 // height in mm
		),
	};

	return drm_connector_helper_get_modes_fixed(connector, &usb2epdiy_mode);
}

static const struct drm_connector_helper_funcs usb2epdiy_conn_helper_funcs = {
	.get_modes = usb2epdiy_conn_get_modes,
};

static const struct drm_connector_funcs usb2epdiy_conn_funcs = {
	.fill_modes = drm_helper_probe_single_connector_modes,
	.destroy = drm_connector_cleanup,
	.reset = drm_atomic_helper_connector_reset,
	.atomic_duplicate_state = drm_atomic_helper_connector_duplicate_state,
	.atomic_destroy_state = drm_atomic_helper_connector_destroy_state,
};

static int usb2epdiy_conn_init(struct usb2epdiy_device *usb2epdiy)
{
	drm_connector_helper_add(&usb2epdiy->conn, &usb2epdiy_conn_helper_funcs);
	return drm_connector_init(&usb2epdiy->drmdev, &usb2epdiy->conn,
							  &usb2epdiy_conn_funcs, DRM_MODE_CONNECTOR_USB);
}

/* ------------------------------------------------------------------ */
/* usb2epdiy (simple) display pipe				      */

static void usb2epdiy_pipe_update(struct drm_simple_display_pipe *pipe,
								  struct drm_plane_state *old_state)
{
	//printk(KERN_INFO "usb2epdiy_pipe_update\n");
	struct drm_plane_state *state = pipe->plane.state;
	struct drm_framebuffer *drm_fb = state->fb;
	struct drm_shadow_plane_state *shadow_plane_state = to_drm_shadow_plane_state(state);
	struct usb2epdiy_device *usb2epdiy = to_usb2epdiy(pipe->crtc.dev);
	struct iosys_map dst;
	unsigned int dst_pitch = 0;
	int ret;
	struct drm_rect fullrect = { 0, 0, usb2epdiy->width, usb2epdiy->height };

	if(!usb2epdiy->pipe_enabled)
		return;

	mutex_lock(&usb2epdiy->fb_lock);
	
	ret = drm_gem_fb_begin_cpu_access(drm_fb, DMA_FROM_DEVICE);
	if(!ret)
	{
		iosys_map_set_vaddr(&dst, usb2epdiy->fb_buf);

		drm_fb_xrgb8888_to_gray8(&dst, &dst_pitch, shadow_plane_state->map, drm_fb, &fullrect);

		drm_gem_fb_end_cpu_access(drm_fb, DMA_FROM_DEVICE);
	} else {
		usb2epdiy_ERR("drm_gem_fb_begin_cpu_access err: %d\n", ret);
	}
	usb2epdiy->fb_is_dirty = true;

	mutex_unlock(&usb2epdiy->fb_lock);

	if(epdiy_is_ready(usb2epdiy)) {
		queue_delayed_work(
			system_long_wq,
			&usb2epdiy->fb_update_work,
			msecs_to_jiffies(0)
		);
	}
}


static void usb2epdiy_pipe_enable(struct drm_simple_display_pipe *pipe,
								  struct drm_crtc_state *crtc_state,
								  struct drm_plane_state *plane_state)
{
	struct usb2epdiy_device *usb2epdiy = to_usb2epdiy(pipe->crtc.dev);
	printk(KERN_INFO "usb2epdiy_pipe_enable\n");
	usb2epdiy->pipe_enabled = true;
	usb2epdiy->last_refresh_id_rcv = usb2epdiy->last_refresh_id;
	usb2epdiy_pipe_update(pipe, NULL);
	mod_delayed_work(
		system_long_wq,
		&usb2epdiy->fb_update_work,
		msecs_to_jiffies(1000)
	);
	mod_delayed_work(
		system_long_wq,
		&usb2epdiy->digest_stream_work,
		msecs_to_jiffies(1100)
	);
}

static void usb2epdiy_pipe_disable(struct drm_simple_display_pipe *pipe)
{
	struct usb2epdiy_device *usb2epdiy = to_usb2epdiy(pipe->crtc.dev);
	printk(KERN_INFO "usb2epdiy_pipe_disable\n");
	usb2epdiy->pipe_enabled = false;
	usb2epdiy->last_refresh_id_rcv = usb2epdiy->last_refresh_id-1;
#if 0
	struct usb2epdiy_device *usb2epdiy = to_usb2epdiy(pipe->crtc.dev);

	usb2epdiy_stop_fb_update(usb2epdiy);
#endif
}

int trigger5_pipe_check(struct drm_simple_display_pipe *pipe,
			struct drm_plane_state *new_plane_state,
			struct drm_crtc_state *new_crtc_state)
{
	return 0;
}


static const struct drm_simple_display_pipe_funcs usb2epdiy_pipe_funcs = {
	.enable = usb2epdiy_pipe_enable,
	.disable = usb2epdiy_pipe_disable,
	.update = usb2epdiy_pipe_update,
	.check = trigger5_pipe_check,
	DRM_GEM_SIMPLE_DISPLAY_PIPE_SHADOW_PLANE_FUNCS,
};

static const uint32_t usb2epdiy_pipe_formats[] = {
	DRM_FORMAT_XRGB8888,
};

static const uint64_t usb2epdiy_pipe_modifiers[] = {
	DRM_FORMAT_MOD_LINEAR,
	DRM_FORMAT_MOD_INVALID
};

/*
 * FIXME: Dma-buf sharing requires DMA support by the importing device.
 *        This function is a workaround to make USB devices work as well.
 *        See todo.rst for how to fix the issue in the dma-buf framework.
 */
static struct drm_gem_object *usb2epdiy_gem_prime_import(
	struct drm_device *drmdev,
	struct dma_buf *dma_buf
) {
	struct usb2epdiy_device *usb2epdiy = to_usb2epdiy(drmdev);

	if (!usb2epdiy->dmadev)
		return ERR_PTR(-ENODEV);

	return drm_gem_prime_import_dev(drmdev, dma_buf, usb2epdiy->dmadev);
}

DEFINE_DRM_GEM_FOPS(usb2epdiy_fops);
// DEFINE_DRM_GEM_DMA_FOPS(usb2epdiy_fops);

static const struct drm_driver usb2epdiy_drm_driver = {
	.driver_features = DRIVER_GEM | DRIVER_ATOMIC | DRIVER_MODESET,

	.name = DRIVER_NAME,
	.desc = DRIVER_DESC,
	.date = DRIVER_DATE,
	.major = DRIVER_MAJOR,
	.minor = DRIVER_MINOR,

	.fops = &usb2epdiy_fops,
	//DRM_GEM_DMA_DRIVER_OPS,
	DRM_GEM_SHMEM_DRIVER_OPS,
	// DRM_GEM_DMA_DRIVER_OPS_VMAP,
	.gem_prime_import = usb2epdiy_gem_prime_import,
};

static const struct drm_mode_config_funcs usb2epdiy_mode_config_funcs = {
	.fb_create = drm_gem_fb_create_with_dirty,
	.atomic_check = drm_atomic_helper_check,
	.atomic_commit = drm_atomic_helper_commit,
};

/* ***** USB to drm_device ***** */

static void digest_stream(struct work_struct *work)
{
	static uint32_t payload_bytes_counter = 0;
	static u8 previous_command = UNKNOWN_COMMAND;
	static u8 current_command = UNKNOWN_COMMAND;
	static int busyctr = 0;
	static uint16_t new_width, new_height;
	struct usb2epdiy_device *usb2epdiy = container_of(
		to_delayed_work(work),
		struct usb2epdiy_device,
		digest_stream_work
	);
	int actual_len;
	int ret = 0;
	ret = usb_bulk_msg(
		usb2epdiy->usbdev,
		usb_rcvbulkpipe(usb2epdiy->usbdev, DATA_RCV_EPT),
		usb2epdiy->usb_rcv_buf,
		USB_RCV_BUF_SIZE,
		&actual_len,
		10  // timeout
	);
	if(!ret) {
		// We received something
		for(int j=0; j<actual_len; j++) {
			u8 byte = usb2epdiy->usb_rcv_buf[j];

			if( byte == COMMAND_FOLLOWS ) {
				previous_command = current_command;
				current_command = COMMAND_FOLLOWS;
				continue;
			}

			if( current_command == COMMAND_FOLLOWS ) {
				switch(byte) {
					case SEND_128_BYTE:
						current_command = previous_command;
						previous_command = UNKNOWN_COMMAND;
						byte = 128;  // Will be processed below
						break;
					case REFRESH_DISPLAY:
						previous_command = current_command;
						current_command = REFRESH_DISPLAY;
						continue;
					case SCREEN_SIZE:
						payload_bytes_counter = 0;
						previous_command = current_command;
						current_command = SCREEN_SIZE;
						continue;
					default:
                    	printk(
							KERN_WARNING "do not know how to interpret this byte after 'command follows' byte: %hhu\n",
							byte
						);
                    	continue;
				};
			}

			switch(current_command) {
				case REFRESH_DISPLAY:
					printk(KERN_INFO "Received synchronization byte: %hhu\n", byte);
					usb2epdiy->last_refresh_id_rcv = byte;
					busyctr = 0;
					previous_command = REFRESH_DISPLAY;
                    current_command = UNKNOWN_COMMAND;
					queue_delayed_work(
						system_long_wq,
						&usb2epdiy->fb_update_work,
						msecs_to_jiffies(0)
					);
					continue;
				case SCREEN_SIZE:
					switch (payload_bytes_counter) {
						case 0: new_width = ((uint16_t)byte); break;
						case 1: new_width |= ((uint16_t)byte)<<8; break;
						case 2: new_height = ((uint16_t)byte); break;
						case 3: new_height |= ((uint16_t)byte)<<8;
							printk(KERN_INFO "Received new screen size: %dx%d\n", new_width, new_height);
							if(usb2epdiy->width != new_width || usb2epdiy->height != new_height) {
								bool init_drm = usb2epdiy->width == 0;
								if(init_drm) {
									usb2epdiy->width = new_width;
									usb2epdiy->height = new_height;

									if(usb2epdiy->fb_buf) kfree(usb2epdiy->fb_buf);
									if(usb2epdiy->fb_buf_outgoing) kfree(usb2epdiy->fb_buf_outgoing);
									if(usb2epdiy->fb_buf_previous) kfree(usb2epdiy->fb_buf_previous);

									usb2epdiy->fb_buf = kmalloc((size_t)new_width * new_height, GFP_KERNEL);
									usb2epdiy->fb_buf_outgoing = kmalloc((size_t)new_width * new_height, GFP_KERNEL);
									usb2epdiy->fb_buf_previous = kmalloc((size_t)new_width * new_height, GFP_KERNEL);

									usb2epdiy->drmdev.mode_config.min_width  = new_width;
									usb2epdiy->drmdev.mode_config.max_width  = new_width;
									usb2epdiy->drmdev.mode_config.min_height = new_height;
									usb2epdiy->drmdev.mode_config.max_height = new_height;

									int ret;
									ret = drm_simple_display_pipe_init(
										&usb2epdiy->drmdev,
										&usb2epdiy->pipe,
										&usb2epdiy_pipe_funcs,
										usb2epdiy_pipe_formats,
										ARRAY_SIZE(usb2epdiy_pipe_formats),
										usb2epdiy_pipe_modifiers,
										&usb2epdiy->conn
									);
									if (ret)
										printk(KERN_WARNING "Failed to initialize simple display pipe\n");
										//goto err_put_device;

									drm_mode_config_reset(&usb2epdiy->drmdev);

									ret = drm_dev_register(&usb2epdiy->drmdev, 0);
									if (ret)
										//goto err_put_device;
										printk(KERN_WARNING "Failed to register drm device\n");

									drm_fbdev_generic_setup(&usb2epdiy->drmdev, 0);

									queue_delayed_work(
										system_long_wq,
										&usb2epdiy->fb_update_work,
										msecs_to_jiffies(0)
									);
								} else {
									printk(KERN_WARNING "Screen size changed, but we are not ready to handle this yet\n");
								}
							}
							continue;
					}
					payload_bytes_counter++;
					continue;

				default:
					if( current_command != previous_command ) {
						printk(KERN_WARNING "Current command is unknown: %hhu\n", current_command);
						previous_command = current_command;
					}
					continue;
			};
		}
	}
	if(!epdiy_is_ready(usb2epdiy)) {
		busyctr++;
		if(busyctr>10) {
			printk(KERN_INFO "Waiting for synchronization byte timed out, maybe we lost it.\n");
			usb2epdiy->last_refresh_id_rcv = usb2epdiy->last_refresh_id;
			busyctr = 0;
			queue_delayed_work(
				system_long_wq,
				&usb2epdiy->fb_update_work,
				msecs_to_jiffies(0)
			);
		}
	} else {
		busyctr = 0;
	}
	queue_delayed_work(
		system_long_wq,
		&usb2epdiy->digest_stream_work,
		msecs_to_jiffies(30)
	);
}

static int usb2epdiy_usb_probe(
	struct usb_interface *usbinterface,
	const struct usb_device_id *id
) {
	struct usb2epdiy_device *usb2epdiy;
	struct drm_device *drmdev;
	int ret;

	printk(KERN_INFO "usb2epdiy_usb_probe\n");

	if (usbinterface->cur_altsetting->desc.bInterfaceNumber != 1)
		return -ENODEV;

	usb2epdiy = devm_drm_dev_alloc(
		&usbinterface->dev,
		&usb2epdiy_drm_driver,
		struct usb2epdiy_device, drmdev
	);
	if (IS_ERR(usb2epdiy))
	{
		printk(KERN_INFO "usb2epdiy could not create drm_device\n");
		return PTR_ERR(usb2epdiy);
	}
	usb2epdiy->pipe_enabled = false;
	usb2epdiy->usbdev = interface_to_usbdev(to_usb_interface(usb2epdiy->drmdev.dev));
	drmdev = &usb2epdiy->drmdev;
	printk(KERN_INFO "usb2epdiy created drm_device\n");

	mutex_init(&usb2epdiy->fb_lock);
	usb2epdiy->width = 0;
	usb2epdiy->height = 0;
	usb2epdiy->fb_buf = NULL;
	usb2epdiy->fb_buf_outgoing = NULL;
	usb2epdiy->fb_buf_previous = NULL;
	usb2epdiy->usb_rcv_buf = kmalloc(USB_RCV_BUF_SIZE+1, GFP_KERNEL);
	usb2epdiy->usb_snd_buf = kmalloc(USB_SND_BUF_SIZE+1, GFP_KERNEL);
	usb2epdiy->usb_snd_buf_filled = 0;
	INIT_DELAYED_WORK(&usb2epdiy->fb_update_work, usb2epdiy_send_fb_worker);
	INIT_DELAYED_WORK(&usb2epdiy->digest_stream_work, digest_stream);
	usb2epdiy->last_refresh_id = 0;
	usb2epdiy->last_refresh_id_rcv = 0;

	usb2epdiy->dmadev = usb_intf_get_dma_device(to_usb_interface(drmdev->dev));
	if (!usb2epdiy->dmadev)
		drm_warn(drmdev, "buffer sharing not supported"); /* not an error */

	ret = drmm_mode_config_init(drmdev);
	if (ret)
		goto err_put_device;

	drmdev->mode_config.min_width  = 0;
	drmdev->mode_config.max_width  = 0;
	drmdev->mode_config.min_height = 0;
	drmdev->mode_config.max_height = 0;
	drmdev->mode_config.prefer_shadow = 1;
	drmdev->mode_config.funcs = &usb2epdiy_mode_config_funcs;

	usb_set_intfdata(usbinterface, drmdev);

	ret = usb2epdiy_conn_init(usb2epdiy);
	if (ret)
		goto err_put_device;

	write_command(usb2epdiy, SCREEN_SIZE, NULL, 0);
	serial_flush(usb2epdiy);
	queue_delayed_work(
		system_long_wq,
		&usb2epdiy->digest_stream_work,
		msecs_to_jiffies(50)
	);

	return 0;

err_put_device:
	if (usb2epdiy->dmadev)
		put_device(usb2epdiy->dmadev);
	return ret;
}

static void usb2epdiy_usb_disconnect(struct usb_interface *interface)
{
	struct drm_device *drmdev = usb_get_intfdata(interface);
	struct usb2epdiy_device *usb2epdiy = to_usb2epdiy(drmdev);

	cancel_delayed_work_sync(&usb2epdiy->digest_stream_work);
	cancel_delayed_work_sync(&usb2epdiy->fb_update_work);

	if(usb2epdiy->dmadev) {
		put_device(usb2epdiy->dmadev);
		usb2epdiy->dmadev = NULL;
	}
	if(usb2epdiy->fb_buf) {
		kfree(usb2epdiy->fb_buf);
		usb2epdiy->fb_buf = NULL;
	}
	if(usb2epdiy->fb_buf_outgoing) {
		kfree(usb2epdiy->fb_buf_outgoing);
		usb2epdiy->fb_buf_outgoing = NULL;
	}
	if(usb2epdiy->fb_buf_previous) {
		kfree(usb2epdiy->fb_buf_previous);
		usb2epdiy->fb_buf_previous = NULL;
	}
	if(usb2epdiy->usb_rcv_buf) {
		kfree(usb2epdiy->usb_rcv_buf);
		usb2epdiy->usb_rcv_buf = NULL;
	}
	if(usb2epdiy->usb_snd_buf) {
		kfree(usb2epdiy->usb_snd_buf);
		usb2epdiy->usb_snd_buf = NULL;
	}
	drm_dev_unplug(drmdev);
	drm_atomic_helper_shutdown(drmdev);
	usb2epdiy->usbdev = NULL;
}

static const struct usb_device_id id_table[] = {
	{USB_DEVICE(0x303a, 0xa299)},
	{},
};
MODULE_DEVICE_TABLE(usb, id_table);

static struct usb_driver usb2epdiy_usb_driver = {
	.name = DRIVER_NAME,
	.probe = usb2epdiy_usb_probe,
	.disconnect = usb2epdiy_usb_disconnect,
	.id_table = id_table,
};

module_usb_driver(usb2epdiy_usb_driver);
MODULE_AUTHOR("Simon Schumann <simon.schumann@web.de>");
MODULE_LICENSE("GPL");
