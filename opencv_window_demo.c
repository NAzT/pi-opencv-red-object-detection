/* 
 * File:   opencv_demo.c
 * Author: Tasanakorn
 *
 * Created on May 22, 2013, 1:52 PM
 */

#include <stdio.h>
#include <stdlib.h>


#include <cv.h>
//#include <opencv2/core/core_c.h>
//#include <opencv2/objdetect/objdetect.hpp>
//#include <opencv2/highgui/highgui.hpp>
//#include <opencv2/imgproc/imgproc.hpp>

#include "bcm_host.h"
#include "interface/vcos/vcos.h"
#include "interface/vcos/vcos.h"

#include "interface/mmal/mmal.h"
#include "interface/mmal/util/mmal_default_components.h"
#include "interface/mmal/util/mmal_connection.h"

#include "vgfont.h"

#define MMAL_CAMERA_PREVIEW_PORT 0
#define MMAL_CAMERA_VIDEO_PORT 1
#define MMAL_CAMERA_CAPTURE_PORT 2

#define VIDEO_FPS 30
//#define VIDEO_WIDTH 1280
//#define VIDEO_HEIGHT 720

#define VIDEO_WIDTH 640
#define VIDEO_HEIGHT 360

#define CENTER_X 320/2
#define CENTER_Y 180/2

typedef struct {
    int video_width;
    int video_height;
    int preview_width;
    int preview_height;
    int opencv_width;
    int opencv_height;
    float video_fps;
    MMAL_POOL_T *camera_video_port_pool;
    CvMemStorage* storage;
    IplImage* image;
    IplImage* image_py;
    IplImage* image_pu;
    IplImage* image_pv;
    IplImage* image_pu_big;
    IplImage* image_pv_big; 
    IplImage* image_3channels;
    IplImage* image_opencv;
    IplImage* image_segmented;
    VCOS_SEMAPHORE_T complete_semaphore;
} PORT_USERDATA;

//int iLowH = 47;
int iLowH = 165;
int iLowS = 150; 
int iLowV = 50;

int iHighH = 255;
int iHighS = 255;
int iHighV = 255;

int servo_x = 150;
int servo_y = 150;

char x_temp[10];
char y_temp[10];

CvMoments *moments;
FILE *fp; 

#define DEBUG 1

void servo_up() { 
    servo_y -= 1; 
    if (DEBUG)
      fprintf(stdout, "go up to %d", servo_y);
    sprintf(y_temp, "0=%d\n", servo_y); 
    fprintf(fp, y_temp);
    fflush(fp); 
}

void servo_down() { 
    servo_y += 1; 
    if (DEBUG)
      fprintf(stdout, "go down to %d", servo_y);
    sprintf(y_temp, "0=%d\n", servo_y); 
    fprintf(fp, y_temp);
    fflush(fp); 
}

static void video_buffer_callback(MMAL_PORT_T *port, MMAL_BUFFER_HEADER_T *buffer) {
    static int frame_count = 0;
    static int frame_post_count = 0;
    static struct timespec t1;
    struct timespec t2;
    MMAL_BUFFER_HEADER_T *new_buffer;
    PORT_USERDATA * userdata = (PORT_USERDATA *) port->userdata;
    MMAL_POOL_T *pool = userdata->camera_video_port_pool;

    if (frame_count == 0) {
        clock_gettime(CLOCK_MONOTONIC, &t1);
    }
    frame_count++; 

    mmal_buffer_header_mem_lock(buffer);
    memcpy(userdata->image_py->imageData, buffer->data, userdata->video_width * userdata->video_height);

    // display image in cv_window
    //cvResize(userdata->image, userdata->image_py, CV_INTER_LINEAR);

    int  w = userdata->video_width;
    int  h = userdata->video_height;
    int h4 = userdata->video_height/4;

    memcpy(userdata->image_pu->imageData,buffer->data+w*h,w*h4); // read U
    memcpy(userdata->image_pv->imageData,buffer->data+w*h+w*h4,w*h4); // read v

    cvResize(userdata->image_pu, userdata->image_pu_big, CV_INTER_NN);
    cvResize(userdata->image_pv, userdata->image_pv_big, CV_INTER_NN);  //CV_INTER_LINEAR looks better but it's slower
    cvMerge(userdata->image_py, userdata->image_pu_big, userdata->image_pv_big, NULL, userdata->image_3channels); 
    cvResize(userdata->image_3channels, userdata->image_opencv, CV_INTER_NN); 

    cvCvtColor(userdata->image_opencv, userdata->image_opencv, CV_YCrCb2RGB);
    cvCvtColor(userdata->image_opencv, userdata->image_opencv, CV_BGR2HSV);

    cvInRangeS(userdata->image_opencv, cvScalar(iLowH, iLowS, iLowV, 0), cvScalar(iHighH, iHighS, iHighV, 0), userdata->image_segmented); //Threshold the image 
        // Calculate the moments to estimate the position of the ball
        moments = (CvMoments*)malloc(sizeof(CvMoments));
        cvMoments(userdata->image_segmented, moments, 1);

        // The actual moment values
        double moment10 = cvGetSpatialMoment(moments, 1, 0);
        double moment01 = cvGetSpatialMoment(moments, 0, 1);
        double area = cvGetCentralMoment(moments, 0, 0);
        //printf("moment10 %f, moment01 %f, area %f\t", moment10, moment01, area);

    // Holding the last and current ball positions
        static int posX = 0;
        static int posY = 0;


        posX = moment10/area;
        posY = moment01/area;

        int lastX = posX;
        int lastY = posY;

        // Print it out for debugging purposes
        //printf("(%d, %d)\n", posX, posY);
        //printf("(%d, %d) ", posX, posY);
        fprintf(stdout, "(%d, %d), (%d, %d)\n", posX, posY, lastX, lastY); 

        if(lastX>0 && lastY>0 && posX>0 && posY>0)
        { 
            int y_err = (posY - CENTER_Y);
            fprintf(stdout, "::: %d :::\n", y_err );
            if ( posY < CENTER_Y) {
              if (frame_count %1 == 0 && y_err < -20)
              {
                servo_down();
              }
            }
            else if ( posY > CENTER_Y) {
              if (frame_count %1 == 0)
              {
                servo_up();
              }
            }
            else { fprintf(stdout, "BUG!!!"); }

            //x_err -= (-posX - CENTER_X)/200;
            //y_err -= (-posY - CENTER_Y)/200; 

            //sprintf(y_temp, "0=%d\n", y_err);
            
            //fprintf(fp, x_temp);
            //fprintf(fp, y_temp);
            //fflush(fp); 
            //printf("===== \n X: %s", x_temp);
            cvLine(userdata->image_segmented, cvPoint(posX, posY), cvPoint(lastX, lastY), cvScalar(0,255,255, 0), 5, 8, 0);
        }
        fflush(stdout);

    cvShowImage("Frame", userdata->image_segmented);

    char c =cvWaitKey(1);

    mmal_buffer_header_mem_unlock(buffer);

    if (vcos_semaphore_trywait(&(userdata->complete_semaphore)) != VCOS_SUCCESS) {
        vcos_semaphore_post(&(userdata->complete_semaphore));
        frame_post_count++;
    }

    if (frame_count % 10 == 0) {
        // print framerate every n frame
        clock_gettime(CLOCK_MONOTONIC, &t2);
        float d = (t2.tv_sec + t2.tv_nsec / 1000000000.0) - (t1.tv_sec + t1.tv_nsec / 1000000000.0);
        float fps = 0.0;

        if (d > 0) {
            fps = frame_count / d;
        } else {
            fps = frame_count;
        }
        userdata->video_fps = fps;
        //printf("  Frame = %d, Frame Post %d, Framerate = %.0f fps \n", frame_count, frame_post_count, fps);
    }

    mmal_buffer_header_release(buffer);
    // and send one back to the port (if still open)
    if (port->is_enabled) {
        MMAL_STATUS_T status;

        new_buffer = mmal_queue_get(pool->queue);

        if (new_buffer)
            status = mmal_port_send_buffer(port, new_buffer);

        if (!new_buffer || status != MMAL_SUCCESS)
            printf("Unable to return a buffer to the video port\n");
    }
}

int main(int argc, char** argv) {
    MMAL_COMPONENT_T *camera = 0;
    MMAL_COMPONENT_T *preview = 0;
    MMAL_ES_FORMAT_T *format;
    MMAL_STATUS_T status;
    MMAL_PORT_T *camera_preview_port = NULL, *camera_video_port = NULL, *camera_still_port = NULL;
    MMAL_PORT_T *preview_input_port = NULL;
    MMAL_POOL_T *camera_video_port_pool;
    MMAL_CONNECTION_T *camera_preview_connection = 0;
    PORT_USERDATA userdata;
    fp = fopen("/dev/servoblaster", "w");
    if (fp == NULL) {
        printf("Error opening file\n");
        exit(0);
    }
    else {
     fprintf(fp, "0=150\n1=150\n");
     fflush(fp); 
    }

    int display_width, display_height;

    printf("Running...\n");

    bcm_host_init();

    userdata.preview_width = VIDEO_WIDTH;
    userdata.preview_height = VIDEO_HEIGHT;
    userdata.video_width = VIDEO_WIDTH;
    userdata.video_height = VIDEO_HEIGHT;
    userdata.opencv_width = VIDEO_WIDTH / 4;
    userdata.opencv_height = VIDEO_HEIGHT / 4; 

    graphics_get_display_size(0, &display_width, &display_height);

    float r_w, r_h;
    r_w = (float) display_width / (float) userdata.opencv_width;
    r_h = (float) display_height / (float) userdata.opencv_height;

    //printf("Display resolution = (%d, %d)\n", display_width, display_height);

    /* setup opencv */
    cvNamedWindow("Frame", 1);

    userdata.storage = cvCreateMemStorage(0);
    userdata.image    = cvCreateImage(cvSize(userdata.video_width, userdata.video_height), IPL_DEPTH_8U, 1);
    userdata.image_py = cvCreateImage(cvSize(userdata.video_width, userdata.video_height), IPL_DEPTH_8U, 1); 

    userdata.image_pu = cvCreateImage(cvSize(userdata.video_width/2, userdata.video_height/2), IPL_DEPTH_8U, 1); 
    userdata.image_pv = cvCreateImage(cvSize(userdata.video_width/2, userdata.video_height/2), IPL_DEPTH_8U, 1); 

    userdata.image_pu_big = cvCreateImage(cvSize(userdata.video_width, userdata.video_height), IPL_DEPTH_8U, 1); 
    userdata.image_pv_big = cvCreateImage(cvSize(userdata.video_width, userdata.video_height), IPL_DEPTH_8U, 1); 

    userdata.image_3channels = cvCreateImage(cvSize(userdata.video_width, userdata.video_height), IPL_DEPTH_8U, 3); 

    userdata.image_opencv =  cvCreateImage(cvSize(userdata.video_width/2, userdata.video_height/2), IPL_DEPTH_8U, 3); 
    userdata.image_segmented = cvCreateImage(cvSize(userdata.video_width/2, userdata.video_height/2), IPL_DEPTH_8U, 1); 

    status = mmal_component_create(MMAL_COMPONENT_DEFAULT_CAMERA, &camera);
    if (status != MMAL_SUCCESS) {
        printf("Error: create camera %x\n", status);
        return -1;
    }

    camera_preview_port = camera->output[MMAL_CAMERA_PREVIEW_PORT];
    camera_video_port = camera->output[MMAL_CAMERA_VIDEO_PORT];
    camera_still_port = camera->output[MMAL_CAMERA_CAPTURE_PORT];

    {
        MMAL_PARAMETER_CAMERA_CONFIG_T cam_config = {
            { MMAL_PARAMETER_CAMERA_CONFIG, sizeof (cam_config)},
            .max_stills_w = VIDEO_WIDTH,
            .max_stills_h = VIDEO_HEIGHT,
            .stills_yuv422 = 0,
            .one_shot_stills = 0,
            .max_preview_video_w = VIDEO_WIDTH,
            .max_preview_video_h = VIDEO_HEIGHT,
            .num_preview_video_frames = 2,
            .stills_capture_circular_buffer_height = 0,
            .fast_preview_resume = 1,
            .use_stc_timestamp = MMAL_PARAM_TIMESTAMP_MODE_RESET_STC
        };

        mmal_port_parameter_set(camera->control, &cam_config.hdr);
    }

    format = camera_video_port->format;

    format->encoding = MMAL_ENCODING_I420;
    format->encoding_variant = MMAL_ENCODING_I420;

    format->es->video.width = userdata.video_width;
    format->es->video.height = userdata.video_height;
    format->es->video.crop.x = 0;
    format->es->video.crop.y = 0;
    format->es->video.crop.width = userdata.video_width;
    format->es->video.crop.height = userdata.video_height;
    format->es->video.frame_rate.num = VIDEO_FPS;
    format->es->video.frame_rate.den = 1;

    camera_video_port->buffer_size = userdata.preview_width * userdata.preview_height * 12 / 8;
    camera_video_port->buffer_num = 1;
    //printf("  Camera video buffer_size = %d\n", camera_video_port->buffer_size);

    status = mmal_port_format_commit(camera_video_port);

    if (status != MMAL_SUCCESS) {
        printf("Error: unable to commit camera video port format (%u)\n", status);
        return -1;
    }

    format = camera_preview_port->format;

    format->encoding = MMAL_ENCODING_OPAQUE;
    format->encoding_variant = MMAL_ENCODING_I420;

    format->es->video.width = userdata.preview_width;
    format->es->video.height = userdata.preview_height;
    format->es->video.crop.x = 0;
    format->es->video.crop.y = 0;
    format->es->video.crop.width = userdata.preview_width;
    format->es->video.crop.height = userdata.preview_height;

    status = mmal_port_format_commit(camera_preview_port);

    if (status != MMAL_SUCCESS) {
        printf("Error: camera viewfinder format couldn't be set\n");
        return -1;
    }

    // create pool form camera video port
    camera_video_port_pool = (MMAL_POOL_T *) mmal_port_pool_create(camera_video_port, camera_video_port->buffer_num, camera_video_port->buffer_size);
    userdata.camera_video_port_pool = camera_video_port_pool;
    camera_video_port->userdata = (struct MMAL_PORT_USERDATA_T *) &userdata;

    status = mmal_port_enable(camera_video_port, video_buffer_callback);
    if (status != MMAL_SUCCESS) {
        printf("Error: unable to enable camera video port (%u)\n", status);
        return -1;
    } 

    status = mmal_component_enable(camera);

    status = mmal_component_create(MMAL_COMPONENT_DEFAULT_VIDEO_RENDERER, &preview);
    if (status != MMAL_SUCCESS) {
        printf("Error: unable to create preview (%u)\n", status);
        return -1;
    }
    preview_input_port = preview->input[0];

    {
        MMAL_DISPLAYREGION_T param;
        param.hdr.id = MMAL_PARAMETER_DISPLAYREGION;
        param.hdr.size = sizeof (MMAL_DISPLAYREGION_T);
        param.set = MMAL_DISPLAY_SET_LAYER;
        param.layer = 0;
        param.set |= MMAL_DISPLAY_SET_FULLSCREEN;
        param.fullscreen = 1;
        status = mmal_port_parameter_set(preview_input_port, &param.hdr);
        if (status != MMAL_SUCCESS && status != MMAL_ENOSYS) {
            printf("Error: unable to set preview port parameters (%u)\n", status);
            return -1;
        }
    }

    status = mmal_connection_create(&camera_preview_connection, camera_preview_port, preview_input_port, MMAL_CONNECTION_FLAG_TUNNELLING | MMAL_CONNECTION_FLAG_ALLOCATION_ON_INPUT);
    if (status != MMAL_SUCCESS) {
        printf("Error: unable to create connection (%u)\n", status);
        return -1;
    }

    status = mmal_connection_enable(camera_preview_connection);
    if (status != MMAL_SUCCESS) {
        printf("Error: unable to enable connection (%u)\n", status);
        return -1;
    }

    if (1) {
        // Send all the buffers to the camera video port
        int num = mmal_queue_length(camera_video_port_pool->queue);
        int q;

        for (q = 0; q < num; q++) {
            MMAL_BUFFER_HEADER_T *buffer = mmal_queue_get(camera_video_port_pool->queue);

            if (!buffer) {
                printf("Unable to get a required buffer %d from pool queue\n", q);
            }

            if (mmal_port_send_buffer(camera_video_port, buffer) != MMAL_SUCCESS) {
                printf("Unable to send a buffer to encoder output port (%d)\n", q);
            }
        } 
    }

    if (mmal_port_parameter_set_boolean(camera_video_port, MMAL_PARAMETER_CAPTURE, 1) != MMAL_SUCCESS) {
        printf("%s: Failed to start capture\n", __func__);
    }

    vcos_semaphore_create(&userdata.complete_semaphore, "mmal_opencv_demo-sem", 0);
    int opencv_frames = 0;
    struct timespec t1;
    struct timespec t2;
    clock_gettime(CLOCK_MONOTONIC, &t1);

    GRAPHICS_RESOURCE_HANDLE img_overlay;
    GRAPHICS_RESOURCE_HANDLE img_overlay2;

    gx_graphics_init("/opt/vc/src/hello_pi/hello_font");

    gx_create_window(0, userdata.opencv_width, userdata.opencv_height, GRAPHICS_RESOURCE_RGBA32, &img_overlay);
    gx_create_window(0, 500, 200, GRAPHICS_RESOURCE_RGBA32, &img_overlay2);
    graphics_resource_fill(img_overlay, 0, 0, GRAPHICS_RESOURCE_WIDTH, GRAPHICS_RESOURCE_HEIGHT, GRAPHICS_RGBA32(0xff, 0, 0, 0x55));
    graphics_resource_fill(img_overlay2, 0, 0, GRAPHICS_RESOURCE_WIDTH, GRAPHICS_RESOURCE_HEIGHT, GRAPHICS_RGBA32(0xff, 0, 0, 0x55));

    graphics_display_resource(img_overlay, 0, 1, 0, 0, display_width, display_height, VC_DISPMAN_ROT0, 1);
    char text[256];

    while (1) {
        if (vcos_semaphore_wait(&(userdata.complete_semaphore)) == VCOS_SUCCESS) {
            opencv_frames++;
            float fps = 0.0;
            if (1) {
                clock_gettime(CLOCK_MONOTONIC, &t2);
                float d = (t2.tv_sec + t2.tv_nsec / 1000000000.0) - (t1.tv_sec + t1.tv_nsec / 1000000000.0);
                if (d > 0) {
                    fps = opencv_frames / d;
                } else {
                    fps = opencv_frames;
                }

                //printf("  OpenCV Frame = %d, Framerate = %.2f fps \n", opencv_frames, fps);
            }

            graphics_resource_fill(img_overlay, 0, 0, GRAPHICS_RESOURCE_WIDTH, GRAPHICS_RESOURCE_HEIGHT, GRAPHICS_RGBA32(0, 0, 0, 0x00));
            graphics_resource_fill(img_overlay2, 0, 0, GRAPHICS_RESOURCE_WIDTH, GRAPHICS_RESOURCE_HEIGHT, GRAPHICS_RGBA32(0, 0, 0, 0x00)); 

            sprintf(text, "Video = %.2f FPS, OpenCV = %.2f FPS", userdata.video_fps, fps);
            graphics_resource_render_text_ext(img_overlay2, 0, 0,
                    GRAPHICS_RESOURCE_WIDTH,
                    GRAPHICS_RESOURCE_HEIGHT,
                    GRAPHICS_RGBA32(0x00, 0xff, 0x00, 0xff), /* fg */
                    GRAPHICS_RGBA32(0, 0, 0, 0x00), /* bg */
                    text, strlen(text), 25);

            graphics_display_resource(img_overlay, 0, 1, 0, 0, display_width, display_height, VC_DISPMAN_ROT0, 1);
            graphics_display_resource(img_overlay2, 0, 2, 0, display_width / 16, GRAPHICS_RESOURCE_WIDTH, GRAPHICS_RESOURCE_HEIGHT, VC_DISPMAN_ROT0, 1);

        }
    }

    cvDestroyWindow("Frame");
    return 0;
}

