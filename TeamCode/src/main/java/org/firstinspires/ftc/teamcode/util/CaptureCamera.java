/* Copyright (c) 2020 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode.util;

import android.graphics.Bitmap;
import android.graphics.ImageFormat;
import android.os.Handler;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.android.util.Size;
import org.firstinspires.ftc.robotcore.external.function.Consumer;
import org.firstinspires.ftc.robotcore.external.function.Continuation;
import org.firstinspires.ftc.robotcore.external.hardware.camera.Camera;
import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraCaptureRequest;
import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraCaptureSequenceId;
import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraCaptureSession;
import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraCharacteristics;
import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraException;
import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraFrame;
import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraManager;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.internal.collections.EvictingBlockingQueue;
import org.firstinspires.ftc.robotcore.internal.network.CallbackLooper;
import org.firstinspires.ftc.robotcore.internal.system.ContinuationSynchronizer;
import org.firstinspires.ftc.robotcore.internal.system.Deadline;

import java.util.concurrent.ArrayBlockingQueue;
import java.util.concurrent.TimeUnit;

/**
 * Encapsulate the capture camera and provides and interface to retrieve capture frames on demand
 */
public class CaptureCamera {

    private static final String TAG = CaptureCamera.class.getName();

    public static final boolean LOGGING_ENABLED = true;

    /**
     * How long we are to wait to be granted permission to use the camera before giving up. Here,
     * we wait indefinitely
     */
    private static final int secondsPermissionTimeout = Integer.MAX_VALUE;

    /**
     * State regarding our interaction with the camera
     */
    private CameraManager mCameraManager;
    private WebcamName mCameraName;
    private Camera mCamera = null;
    private CameraCaptureSession mCameraCaptureSession = null;

    /**
     * The queue into which all frames from the camera are placed as they become available.
     * Frames which are not processed by the OpMode are automatically discarded.
     */
    private EvictingBlockingQueue<Bitmap> mFrameQueue;

    /**
     * A utility object that indicates where the asynchronous callbacks from the camera
     * infrastructure are to run.
     **/
    private Handler mCallbackHandler;

    private OpMode mOpMode = null;

    public static final String WEBCAM_NAME = "webcam";

    private ICaptureCameraListener mListener = null;

    /**
     * Must be called from initialization opMode. Throws Exception if it cannot find camera
     * @param opMode the owning opMode
     * @param listener single listener for callbacks on each new frame.
     */
    public void init(OpMode opMode,ICaptureCameraListener listener) throws Exception {
        mOpMode = opMode;
        mListener = listener;
        mCallbackHandler =  CallbackLooper.getDefault().getHandler();
        mCameraManager = ClassFactory.getInstance().getCameraManager();

        mCameraName = mOpMode.hardwareMap.get(WebcamName.class, WEBCAM_NAME);

        initializeFrameQueue(2);

        boolean success = openCamera();
        if (!success) {
            throw new Exception("camera not found or permission to use not granted:"+mCameraName);
        };

    }

    /**
     * Must be called from loop method of opMode repeatedly to get frames
     */
    public void serviceCaptureCamera() {
        Bitmap bmp = mFrameQueue.poll();
 //       log("serviceCaptureCamera:"+bmp);
        if (bmp != null) {
            if (mListener != null) {
                mListener.onNewFrame(bmp);
            }
            // Recycle to avoid memory leaks
            bmp.recycle();
        }
    }

    //----------------------------------------------------------------------------------------------
    // Camera operations
    //----------------------------------------------------------------------------------------------

    private void initializeFrameQueue(int capacity) {
        /** The frame queue will automatically throw away bitmap frames if they are not processed
         * quickly by the OpMode. This avoids a buildup of frames in memory */
        mFrameQueue = new EvictingBlockingQueue<Bitmap>(new ArrayBlockingQueue<Bitmap>(capacity));
        mFrameQueue.setEvictAction(new Consumer<Bitmap>() {
            @Override public void accept(Bitmap frame) {
                // RobotLog.ii(TAG, "frame recycled w/o processing");
                frame.recycle(); // not strictly necessary, but helpful
            }
        });
    }

    private boolean openCamera() {
        if (mCamera != null)
            return true; // be idempotent
        Deadline deadline = new Deadline(secondsPermissionTimeout, TimeUnit.SECONDS);
        mCamera = mCameraManager.requestPermissionAndOpenCamera(deadline, mCameraName, null);
        if (mCamera == null){
            return false;
        }
        else{
            return true;
        }
    }

    /**
     * Called to startCapture.  Camera must have been opened successfully at init.
     * If capture is already running or camera is not open, then returns without action.
     */
    public void startCapture() {
        if (mCameraCaptureSession != null) return; // be idempotent
        if (mCamera == null)
            return;  // camera init error

        /** YUY2 is supported by all Webcams, per the USB Webcam standard: See "USB Device Class Definition
         * for Video Devices: Uncompressed Payload, Table 2-1". Further, often this is the *only*
         * image format supported by a camera */
        final int imageFormat = ImageFormat.YUY2;

        /** Verify that the image is supported, and fetch size and desired frame rate if so */
        CameraCharacteristics cameraCharacteristics = mCameraName.getCameraCharacteristics();
        if (!contains(cameraCharacteristics.getAndroidFormats(), imageFormat)) {
            log("image format not supported");
            return;
        }
        final Size size = cameraCharacteristics.getDefaultSize(imageFormat);
        final int fps = cameraCharacteristics.getMaxFramesPerSecond(imageFormat, size);

        /** Some of the logic below runs asynchronously on other threads. Use of the synchronizer
         * here allows us to wait in this method until all that asynchrony completes before returning. */
        final ContinuationSynchronizer<CameraCaptureSession> synchronizer = new ContinuationSynchronizer<>();
        try {
            /** Create a session in which requests to capture frames can be made */
            mCamera.createCaptureSession(Continuation.create(mCallbackHandler, new CameraCaptureSession.StateCallbackDefault() {
                @Override public void onConfigured( CameraCaptureSession session) {
                    try {
                        /** The session is ready to go. Start requesting frames */
                        final CameraCaptureRequest captureRequest = mCamera.createCaptureRequest(imageFormat, size, fps);
                        session.startCapture(captureRequest,
                                new CameraCaptureSession.CaptureCallback() {
                                    @Override public void onNewFrame( CameraCaptureSession session, CameraCaptureRequest request,  CameraFrame cameraFrame) {
                                        /** A new frame is available. The frame data has <em>not</em> been copied for us, and we can only access it
                                         * for the duration of the callback. So we copy here manually. */
                                        Bitmap bmp = captureRequest.createEmptyBitmap();
                                        cameraFrame.copyToBitmap(bmp);
                                        mFrameQueue.offer(bmp);
                                    }
                                },
                                Continuation.create(mCallbackHandler, new CameraCaptureSession.StatusCallback() {
                                    @Override public void onCaptureSequenceCompleted( CameraCaptureSession session, CameraCaptureSequenceId cameraCaptureSequenceId, long lastFrameNumber) {
                                        RobotLog.ii(TAG, "capture sequence %s reports completed: lastFrame=%d", cameraCaptureSequenceId, lastFrameNumber);
                                        //log("onCaptureSequenceCompleted called: id="+cameraCaptureSequenceId+" frame#:"+lastFrameNumber);
                                    }
                                })
                        );

                        synchronizer.finish(session);
                    } catch (CameraException|RuntimeException e) {
                        RobotLog.ee(TAG, e, "exception starting capture");
                        session.close();
                        synchronizer.finish(null);
                    }
                }
            }));
        } catch (CameraException|RuntimeException e) {
            RobotLog.ee(TAG, e, "exception starting camera");
            synchronizer.finish(null);
        }

        /** Wait for all the asynchrony to complete */
        try {
            synchronizer.await();
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }

        /** Retrieve the created session. This will be null on error. */
        mCameraCaptureSession = synchronizer.getValue();
    }

    /**
     * Called to stop capture but still leaves the camera running
     * To restart capture call startCapture
     */
    public void stopCapture() {
        if (mCameraCaptureSession != null) {
            mCameraCaptureSession.stopCapture();
            mCameraCaptureSession.close();
            mCameraCaptureSession = null;
        }
    }

    /**
     * Returns state of capture
     */
    public boolean isCaptureActive(){
        return (mCameraCaptureSession != null);
    }
    /**
     * Must be called on stop to close resources.  This method closes the
     * camera which cannot be restarted
     */
    public void stop() {
        stopCapture();
        if (mCamera != null) {
            mCamera.close();
            mCamera = null;
        }

    }

    //----------------------------------------------------------------------------------------------
    // Utilities
    //----------------------------------------------------------------------------------------------

    private boolean contains(int[] array, int value) {
        for (int i : array) {
            if (i == value) return true;
        }
        return false;
    }


    private void log(String message){
        if (LOGGING_ENABLED){
            mOpMode.telemetry.addLine(message);
            mOpMode.telemetry.update();
        }
    }

}
