package com.example.arcoredemo

import android.Manifest
import android.app.Activity
import android.bluetooth.BluetoothAdapter
import android.bluetooth.BluetoothManager
import android.content.Context
import android.content.DialogInterface
import android.content.pm.PackageManager
import android.graphics.ImageFormat
import android.graphics.SurfaceTexture
import android.hardware.camera2.CameraAccessException
import android.hardware.camera2.CameraCaptureSession
import android.hardware.camera2.CameraCharacteristics
import android.hardware.camera2.CameraDevice
import android.hardware.camera2.CameraManager
import android.hardware.camera2.CaptureFailure
import android.hardware.camera2.CaptureRequest
import android.hardware.camera2.TotalCaptureResult
import android.media.Image
import android.media.ImageReader
import android.opengl.GLES20
import android.opengl.GLSurfaceView
import android.os.*
import android.util.Log
import android.view.Surface
import androidx.appcompat.app.AlertDialog
import androidx.appcompat.app.AppCompatActivity
import androidx.core.app.ActivityCompat
import androidx.core.content.ContextCompat
import com.google.ar.core.ArCoreApk.InstallStatus
import com.google.ar.core.ArCoreApk.Availability
import com.example.arcoredemo.common.helpers.CameraPermissionHelper
import com.example.arcoredemo.common.helpers.DisplayRotationHelper
import com.example.arcoredemo.common.helpers.FullScreenHelper
import com.example.arcoredemo.common.helpers.SnackbarHelper
import com.example.arcoredemo.common.helpers.TrackingStateHelper
import com.example.arcoredemo.common.rendering.*
import com.google.ar.core.exceptions.CameraNotAvailableException
import com.google.ar.core.exceptions.UnavailableException
import java.io.File
import java.io.FileReader
import java.io.FileWriter
import java.io.IOException
import java.util.concurrent.atomic.AtomicBoolean
import javax.microedition.khronos.egl.EGLConfig
import javax.microedition.khronos.opengles.GL10
import android.widget.*
import com.example.arcoredemo.common.*
import com.google.ar.core.*
import java.nio.BufferUnderflowException
import java.text.SimpleDateFormat
import java.util.*
import kotlin.math.max
import kotlin.math.min
import kotlin.system.measureTimeMillis


//useful android constants
private const val ENABLE_BLUETOOTH_REQUEST_CODE = 1
private const val LOCATION_PERMISSION_REQUEST_CODE = 2

//BLE status change constants
private const val DEVICE_FOUND=0
private const val CONNECTION_START=1
private const val CONNECTION_ERROR=2
private const val CONNECTION_COMPLETE=3
private const val LIDAR_CHARACTERISTIC_CHANGED=4
private const val LIDAR_CHARACTERISTIC_READ=5
private const val LIDAR_CHARACTERISTIC_PARSED=6
private const val LIDAR_CHARACTERISTIC_DRAWABLE=7

//lidar data constants
private const val ANGLE_OFFSET=180

//cloud anchor test constants
private const val CLOUD_ANCHOR_ID="ua-52947778a92664715ecb5a67664744d2" //casa padova
//private const val CLOUD_ANCHOR_ID="ua-aed91188d6f3e7c22c864dad47165c88" //klaxon ufficio
//private const val CLOUD_ANCHOR_ID="ua-7221a0cec8445e204021537e076033d3" //klaxon magazzino NO
//private const val CLOUD_ANCHOR_ID="ua-064db3c7bbdaee14d72b57c3013f1c65" //klaxon magazzino



class MainActivity() : AppCompatActivity(), GLSurfaceView.Renderer,
    ImageReader.OnImageAvailableListener, SurfaceTexture.OnFrameAvailableListener {
    // Whether the app is currently in AR mode. Initial value determines initial state.
    private var arMode = true

    // Whether the app has just entered non-AR mode.
    private val isFirstFrameWithoutArcore = AtomicBoolean(true)

    // GL Surface used to draw camera preview image.
    private lateinit var surfaceView: GLSurfaceView

    // Text view for displaying on screen status message.
    private lateinit var statusTextView: TextView

    // Linear layout that contains preview image and status text.
    private lateinit var imageTextLinearLayout: LinearLayout

    // ARCore session that supports camera sharing.
    private lateinit var sharedSession: Session

    // Camera capture session. Used by both non-AR and AR modes.
    private lateinit var captureSession: CameraCaptureSession

    // Reference to the camera system service.
    private val cameraManager: CameraManager by lazy{
        this.getSystemService(CAMERA_SERVICE) as CameraManager
    }

    private val characteristics:CameraCharacteristics by lazy{
        cameraManager.getCameraCharacteristics(cameraId)
    }

    // A list of CaptureRequest keys that can cause delays when switching between AR and non-AR modes.
    private lateinit var keysThatCanCauseCaptureDelaysWhenModified: List<CaptureRequest.Key<*>>

    // Camera device. Used by both non-AR and AR modes.
    private lateinit var cameraDevice: CameraDevice

    // Looper handler thread.
    private lateinit var backgroundThread: HandlerThread

    // Looper handler.
    private lateinit var backgroundHandler: Handler

    // ARCore shared camera instance, obtained from ARCore session that supports sharing.
    private lateinit var sharedCamera: SharedCamera

    // Camera ID for the camera used by ARCore.
    private lateinit var cameraId: String

    // Ensure GL surface draws only occur when new frames are available.
    private val shouldUpdateSurfaceTexture = AtomicBoolean(false)

    // Whether ARCore is currently active.
    private var arcoreActive = false

    // Whether the GL surface has been created.
    private var surfaceCreated = false

    /**
     * Whether an error was thrown during session creation.
     */
    private var errorCreatingSession = false

    // Camera preview capture request builder
    private lateinit var previewCaptureRequestBuilder: CaptureRequest.Builder

    // Image reader that continuously processes CPU images.
    private lateinit var cpuImageReader: ImageReader

    // Total number of CPU images processed.
    private var cpuImagesProcessed = 0

    // Various helper classes, see hello_ar_java sample to learn more.
    private val messageSnackbarHelper: SnackbarHelper = SnackbarHelper()
    private lateinit var displayRotationHelper: DisplayRotationHelper
    private val trackingStateHelper: TrackingStateHelper = TrackingStateHelper(this)
    //private lateinit var tapHelper: TapHelper

    // Renderers, see hello_ar_java sample to learn more.
    private val backgroundRenderer = BackgroundRenderer()
    private val virtualObject = ObjectRenderer()
    private val virtualObjectShadow = ObjectRenderer()
    private val planeRenderer = PlaneRenderer()
    private val pointCloudRenderer = PointCloudRenderer()

    private val myPointCloudRenderer = MyPointCloudRenderer()
    private val myPointCloud= LidarPointCloud(ANGLE_OFFSET)


    // Temporary matrix allocated here to reduce number of allocations for each frame.
    private val anchorMatrix = FloatArray(16)

    // Anchors created from taps, see hello_ar_java sample to learn more.
    private val anchors = ArrayList<ColoredAnchor>()
    private val automatorRun = AtomicBoolean(false)

    // Prevent any changes to camera capture session after CameraManager.openCamera() is called, but
    // before camera device becomes active.
    private var captureSessionChangesPossible = true

    // A check mechanism to ensure that the camera closed properly so that the app can safely exit.
    private val safeToExitApp = ConditionVariable()

    private class ColoredAnchor(val anchor: Anchor, val color: FloatArray)

    //BLE variables

    private lateinit var bleLidarConnection: BLELidarConnection

    private val bluetoothManager by lazy{
        getSystemService(Context.BLUETOOTH_SERVICE) as BluetoothManager
    }

    //get bluetooth adapter manager to check bluetooth status
    private val bluetoothAdapter: BluetoothAdapter by lazy {
        bluetoothManager.adapter
    }

    //variable to check if fine location permission has been granted
    private val isLocationPermissionGranted
        get() = hasPermission(Manifest.permission.ACCESS_FINE_LOCATION)


    private var currentSpeed=-1
    private var currentDirection=-1
    private var connectionStatus=false
    private val progressBar:ProgressBar by lazy{
        findViewById(R.id.progressBar)
    }


    //composite anchored cloud
    private var anchorRequested=false
    private var addToCloudRequested=false
    private var establishCloudAnchor=false
    private var resolveCloudAnchor=false
    private lateinit var compositeCloud:CompositePointCloud
    private var lastLidarPoints= mutableListOf<LidarPoint>()

    var saveCounter=0

    private lateinit var cloudAnchor:Anchor




    // Camera device state callback.
    private val cameraDeviceCallback: CameraDevice.StateCallback =
        object : CameraDevice.StateCallback() {
            override fun onOpened(cameraDevice: CameraDevice) {
                Log.d(TAG, "Camera device ID " + cameraDevice.id + " opened.")
                this@MainActivity.cameraDevice = cameraDevice
                createCameraPreviewSession()
            }

            override fun onClosed(cameraDevice: CameraDevice) {
                Log.d(TAG, "Camera device ID " + cameraDevice.id + " closed.")
                //this@MainActivity.cameraDevice = null TODO
                safeToExitApp.open()
            }

            override fun onDisconnected(cameraDevice: CameraDevice) {
                Log.w(TAG, "Camera device ID " + cameraDevice.id + " disconnected.")
                cameraDevice.close()
                //this@MainActivity.cameraDevice = null TODO
            }

            override fun onError(cameraDevice: CameraDevice, error: Int) {
                Log.e(TAG, "Camera device ID " + cameraDevice.id + " error " + error)
                cameraDevice.close()
                //this@MainActivity.cameraDevice = null TODO
                // Fatal error. Quit application.
                finish()
            }
        }

    // Repeating camera capture session state callback.
    var cameraSessionStateCallback: CameraCaptureSession.StateCallback =
        object : CameraCaptureSession.StateCallback() {
            // Called when the camera capture session is first configured after the app
            // is initialized, and again each time the activity is resumed.
            override fun onConfigured(session: CameraCaptureSession) {
                Log.d(TAG, "Camera capture session configured.")
                captureSession = session
                if (arMode) {
                    setRepeatingCaptureRequest()
                    // Note, resumeARCore() must be called in onActive(), not here.
                } else {
                    // Calls `setRepeatingCaptureRequest()`.
                    //resumeCamera2()
                }
            }

            override fun onSurfacePrepared(
                session: CameraCaptureSession, surface: Surface
            ) {
                Log.d(TAG, "Camera capture surface prepared.")
            }

            override fun onReady(session: CameraCaptureSession) {
                Log.d(TAG, "Camera capture session ready.")
            }

            @Synchronized
            override fun onActive(session: CameraCaptureSession) {
                Log.d(TAG, "Camera capture session active.")
                if (arMode && !arcoreActive) {
                    resumeARCore()
                }
                synchronized(this@MainActivity) {
                    captureSessionChangesPossible = true
                    (this as Object).notify()//TODO sistemare
                }
                //updateSnackbarMessage()
            }

            override fun onCaptureQueueEmpty(session: CameraCaptureSession) {
                Log.w(TAG, "Camera capture queue empty.")
            }

            override fun onClosed(session: CameraCaptureSession) {
                Log.d(TAG, "Camera capture session closed.")
            }

            override fun onConfigureFailed(session: CameraCaptureSession) {
                Log.e(TAG, "Failed to configure camera capture session.")
            }
        }

    // Repeating camera capture session capture callback.
    private val cameraCaptureCallback: CameraCaptureSession.CaptureCallback = object : CameraCaptureSession.CaptureCallback() {
        override fun onCaptureCompleted(
            session: CameraCaptureSession,
            request: CaptureRequest,
            result: TotalCaptureResult
        ) {
            shouldUpdateSurfaceTexture.set(true)
        }

        override fun onCaptureBufferLost(
            session: CameraCaptureSession,
            request: CaptureRequest,
            target: Surface,
            frameNumber: Long
        ) {
            Log.e(
                TAG,
                "onCaptureBufferLost: $frameNumber"
            )
        }

        override fun onCaptureFailed(
            session: CameraCaptureSession,
            request: CaptureRequest,
            failure: CaptureFailure
        ) {
            Log.e(TAG, "onCaptureFailed: " + failure.frameNumber + " " + failure.reason)
        }

        override fun onCaptureSequenceAborted(
            session: CameraCaptureSession, sequenceId: Int
        ) {
            Log.e(
                TAG,
                "onCaptureSequenceAborted: $sequenceId $session"
            )
        }
    }

    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)
        setContentView(R.layout.activity_main)
        val extraBundle = intent.extras
        if (extraBundle != null && 1 == extraBundle.getShort(AUTOMATOR_KEY, AUTOMATOR_DEFAULT)
                .toInt()
        ) {
            automatorRun.set(true)
        }

        // GL surface view that renders camera preview image.
        surfaceView = findViewById(R.id.glsurfaceview)
        surfaceView.preserveEGLContextOnPause = true
        surfaceView.setEGLContextClientVersion(2)
        surfaceView.setEGLConfigChooser(8, 8, 8, 8, 16, 0)
        surfaceView.setRenderer(this)
        surfaceView.renderMode = GLSurfaceView.RENDERMODE_CONTINUOUSLY

        // Helpers, see hello_ar_java sample to learn more.
        displayRotationHelper = DisplayRotationHelper(this)
        //tapHelper = TapHelper(this)
        //surfaceView.setOnTouchListener(tapHelper)
        imageTextLinearLayout = findViewById(R.id.image_text_layout)
        statusTextView = findViewById(R.id.text_view)

        // Switch to allow pausing and resuming of ARCore.
        //val arcoreSwitch = findViewById<Switch>(R.id.arcore_switch)
        // Ensure initial switch position is set based on initial value of `arMode` variable.
        //arcoreSwitch.isChecked = arMode
        /*arcoreSwitch.setOnCheckedChangeListener { _, checked ->
            Log.i(TAG, "Switching to " + (if (checked) "AR" else "non-AR") + " mode.")
            if (checked) {
                arMode = true
                resumeARCore()
            } else {
                arMode = false
                pauseARCore()
                resumeCamera2()
            }
            updateSnackbarMessage()
        }*/
        messageSnackbarHelper.setMaxLines(4)
        //updateSnackbarMessage()

        //TODO popolo di punti finti la mia point cloud
        /*for(i in 0..359 ){
            myPointCloud.addPoint(LidarPoint(i,0.3f,1))
        }*/

        //create a ble connection
        bleLidarConnection=object : BLELidarConnection(bluetoothManager,this){
            override fun onConnectionStatusChange(status:Int){
                //TODO gestire tutte le risposte dalla classe
                Log.d("BLELidarConnection","Status: $status")
                when(status){
                    LIDAR_CHARACTERISTIC_PARSED->{
                        myPointCloud.addPoints(bleLidarConnection.lidarPointArray)
                        currentSpeed=bleLidarConnection.currentSpeed//TODO debug
                        currentDirection=bleLidarConnection.currentDirection
                        //startRadarLoop()
                        //drawLidarData(bleLidarConnection.returnLidarPointList())
                        //speedView.text=bleLidarConnection.currentSpeed.toString()
                        //directionView.text=bleLidarConnection.currentDirection.toString()
                    }

                    LIDAR_CHARACTERISTIC_DRAWABLE->{
                        //drawLidarData(bleLidarConnection.returnLidarPointList())
                    }

                    CONNECTION_ERROR->{
                        connectionStatus=false
                    }

                    CONNECTION_COMPLETE->{
                        connectionStatus=true
                        runOnUiThread {
                            //scanButton.text="Connected"
                        }
                    }
                    else->{
                        null
                    }
                }
            }

            override fun onDataParsed(data: List<LidarPoint>) {
                lastLidarPoints=data.toMutableList()
            }
        }

        try{
            startConnection()
        }catch(e:Exception){
            Log.d("ASD","ASD $e")
        }




        val buttonOne : Button = findViewById(R.id.button)
        buttonOne.setOnClickListener{

            Handler(Looper.getMainLooper()).postDelayed(
                {
                    compositeCloud.lock=true
                    saveToPLY()
                    compositeCloud.lock=false
                },
                1000
            )

            //readFromFile()
        }

        val buttonAnchor : Button = findViewById(R.id.buttonAnchor)
        buttonAnchor.setOnClickListener{
            anchorRequested=true
        }

        /*val buttonAddToCloud : Button = findViewById(R.id.buttonAddPoints)
        buttonAddToCloud.setOnClickListener{
            addToCloudRequested=true
        }*/

        val buttonEstablishCloudAnchor : Button = findViewById(R.id.buttonEstablishCloud)
        buttonEstablishCloudAnchor.setOnClickListener{
            establishCloudAnchor=true
        }

        val buttonResolveCloudAnchor : Button = findViewById(R.id.buttonResolveCloud)
        buttonResolveCloudAnchor.setOnClickListener{
            resolveCloudAnchor=true
        }

    }

    private fun startConnection(){//TODO check isscanning and give feedback
        if (isLocationPermissionGranted && bluetoothAdapter.isEnabled) {
            bleLidarConnection.startBleScan()
        }else {
            Log.e("StartConnectionError","Location permission not granted or bluetooth off")
        }
    }

    override fun onDestroy() {
        if (::sharedSession.isInitialized) {
            // Explicitly close ARCore Session to release native resources.
            // Review the API reference for important considerations before calling close() in apps with
            // more complicated lifecycle requirements:
            // https://developers.google.com/ar/reference/java/arcore/reference/com/google/ar/core/Session#close()
            sharedSession.close()
            //sharedSession = null TODO
        }
        super.onDestroy()
    }

    @Synchronized
    private fun waitUntilCameraCaptureSessionIsActive() {
        while (!captureSessionChangesPossible) {
            try {
                (this as Object).wait() //TODO cambiare https://stackoverflow.com/questions/44589669/correctly-implementing-wait-and-notify-in-kotlin
            } catch (e: InterruptedException) {
                Log.e(TAG,"Unable to wait for a safe time to make changes to the capture session", e)
            }
        }
    }

    override fun onResume() {
        super.onResume()
        waitUntilCameraCaptureSessionIsActive()
        startBackgroundThread()
        surfaceView.onResume()

        // When the activity starts and resumes for the first time, openCamera() will be called
        // from onSurfaceCreated(). In subsequent resumes we call openCamera() here.
        if (surfaceCreated) {
            openCamera()
        }
        displayRotationHelper.onResume()
    }

    public override fun onPause() {
        shouldUpdateSurfaceTexture.set(false)
        surfaceView.onPause()
        waitUntilCameraCaptureSessionIsActive()
        displayRotationHelper.onPause()
        if (arMode) {
            pauseARCore()
        }
        closeCamera()
        stopBackgroundThread()
        super.onPause()
    }

    /*private fun resumeCamera2() {
        setRepeatingCaptureRequest()
        sharedCamera.surfaceTexture.setOnFrameAvailableListener(this)
    }*/

    private fun resumeARCore() {
        // Ensure that session is valid before triggering ARCore resume. Handles the case where the user
        // manually uninstalls ARCore while the app is paused and then resumes.
        if (!::sharedSession.isInitialized) {
            return
        }
        if (!arcoreActive) {
            try {
                // To avoid flicker when resuming ARCore mode inform the renderer to not suppress rendering
                // of the frames with zero timestamp.
                backgroundRenderer.suppressTimestampZeroRendering(false)
                // Resume ARCore.
                sharedSession.resume()
                arcoreActive = true
                //updateSnackbarMessage()

                // Set capture session callback while in AR mode.
                sharedCamera.setCaptureCallback(cameraCaptureCallback, backgroundHandler)
            } catch (e: CameraNotAvailableException) {
                Log.e(TAG, "Failed to resume ARCore session", e)
                return
            }
        }
    }

    private fun pauseARCore() {
        if (arcoreActive) {
            // Pause ARCore.
            sharedSession.pause()
            isFirstFrameWithoutArcore.set(true)
            arcoreActive = false
            //updateSnackbarMessage()
        }
    }

    /*private fun updateSnackbarMessage() {
        messageSnackbarHelper.showMessage(
            this,
            if (arcoreActive) "ARCore is active.\nSearch for plane, then tap to place a 3D model." else "ARCore is paused.\nCamera effects enabled."
        )
    }*/

    // Called when starting non-AR mode or switching to non-AR mode.
    // Also called when app starts in AR mode, or resumes in AR mode.
    private fun setRepeatingCaptureRequest() {
        try {
            setCameraEffects(previewCaptureRequestBuilder)
            captureSession.setRepeatingRequest(
                previewCaptureRequestBuilder.build(), cameraCaptureCallback, backgroundHandler
            )
        } catch (e: CameraAccessException) {
            Log.e(TAG, "Failed to set repeating request", e)
        }
    }

    private fun createCameraPreviewSession() {
        try {
            sharedSession.setCameraTextureName(backgroundRenderer.textureId)
            sharedCamera.surfaceTexture.setOnFrameAvailableListener(this)

            // Create an ARCore compatible capture request using `TEMPLATE_RECORD`.
            previewCaptureRequestBuilder =
                cameraDevice.createCaptureRequest(CameraDevice.TEMPLATE_RECORD)

            // Build surfaces list, starting with ARCore provided surfaces.
            val surfaceList = sharedCamera.arCoreSurfaces

            // Add a CPU image reader surface. On devices that don't support CPU image access, the image
            // may arrive significantly later, or not arrive at all.
            surfaceList.add(cpuImageReader.surface)

            // Surface list should now contain three surfaces:
            // 0. sharedCamera.getSurfaceTexture()
            // 1. …
            // 2. cpuImageReader.getSurface()

            // Add ARCore surfaces and CPU image surface targets.
            for (surface: Surface in surfaceList) {
                previewCaptureRequestBuilder.addTarget(surface)
            }

            // Wrap our callback in a shared camera callback.
            val wrappedCallback = sharedCamera.createARSessionStateCallback(
                cameraSessionStateCallback,
                backgroundHandler
            )

            // Create camera capture session for camera preview using ARCore wrapped callback.
            cameraDevice.createCaptureSession(surfaceList, wrappedCallback, backgroundHandler)//TODO config?
        } catch (e: CameraAccessException) {
            Log.e(TAG, "CameraAccessException", e)
        }
    }

    // Start background handler thread, used to run callbacks without blocking UI thread.
    private fun startBackgroundThread() {
        backgroundThread = HandlerThread("sharedCameraBackground")
        backgroundThread.start()
        backgroundHandler = Handler(backgroundThread.looper)
    }

    // Stop background handler thread.
    private fun stopBackgroundThread() {
        if (::backgroundThread.isInitialized) {
            backgroundThread.quitSafely()
            try {
                backgroundThread.join()
                //backgroundThread = null TODO
                //backgroundHandler = null TODO
            } catch (e: InterruptedException) {
                Log.e(TAG, "Interrupted while trying to join background handler thread", e)
            }
        }
    }

    // Perform various checks, then open camera device and create CPU image reader.
    private fun openCamera() {
        // Don't open camera if already opened.
        if (::cameraDevice.isInitialized) {
            return
        }

        // Verify CAMERA_PERMISSION has been granted.
        if (!CameraPermissionHelper.hasCameraPermission(this)) {
            CameraPermissionHelper.requestCameraPermission(this)
            return
        }

        // Make sure that ARCore is installed, up to date, and supported on this device.
        if (!isARCoreSupportedAndUpToDate) {
            return
        }
        if (!::sharedSession.isInitialized) {
            try {
                // Create ARCore session that supports camera sharing.
                sharedSession = Session(this, EnumSet.of(Session.Feature.SHARED_CAMERA))
            } catch (e: Exception) {
                errorCreatingSession = true
                messageSnackbarHelper.showError(
                    this, "Failed to create ARCore session that supports camera sharing"
                )
                Log.e(TAG, "Failed to create ARCore session that supports camera sharing", e)
                return
            }
            errorCreatingSession = false

            // Enable auto focus mode while ARCore is running.
            val config = sharedSession.config
            config.focusMode = Config.FocusMode.AUTO
            config.depthMode=Config.DepthMode.AUTOMATIC
            config.planeFindingMode=Config.PlaneFindingMode.HORIZONTAL_AND_VERTICAL
            config.cloudAnchorMode=Config.CloudAnchorMode.ENABLED

            sharedSession.configure(config)
        }

        // Store the ARCore shared camera reference.
        sharedCamera = sharedSession.sharedCamera

        // Store the ID of the camera used by ARCore.
        cameraId = sharedSession.cameraConfig.cameraId

        // Use the currently configured CPU image size.
        val desiredCpuImageSize = sharedSession.cameraConfig.imageSize
        cpuImageReader = ImageReader.newInstance(
            desiredCpuImageSize.width,
            desiredCpuImageSize.height,
            ImageFormat.YUV_420_888,
            2
        )
        cpuImageReader.setOnImageAvailableListener(this, backgroundHandler)

        // When ARCore is running, make sure it also updates our CPU image surface.
        sharedCamera.setAppSurfaces(cameraId, listOf(cpuImageReader.surface))
        try {

            // Wrap our callback in a shared camera callback.
            val wrappedCallback =
                sharedCamera.createARDeviceStateCallback(cameraDeviceCallback, backgroundHandler)

            // Store a reference to the camera system service.
            //cameraManager = this.getSystemService(CAMERA_SERVICE) as CameraManager //SPOSTATO

            // Get the characteristics for the ARCore camera.
            //val characteristics = cameraManager.getCameraCharacteristics(cameraId) //SPOSTATO

            // On Android P and later, get list of keys that are difficult to apply per-frame and can
            // result in unexpected delays when modified during the capture session lifetime.
            if (Build.VERSION.SDK_INT >= 28) {
                keysThatCanCauseCaptureDelaysWhenModified = characteristics.availableSessionKeys
                if (!::keysThatCanCauseCaptureDelaysWhenModified.isInitialized) {
                    // Initialize the list to an empty list if getAvailableSessionKeys() returns null.
                    keysThatCanCauseCaptureDelaysWhenModified = ArrayList()
                }
            }else{
                keysThatCanCauseCaptureDelaysWhenModified = ArrayList()
            }

            // Prevent app crashes due to quick operations on camera open / close by waiting for the
            // capture session's onActive() callback to be triggered.
            captureSessionChangesPossible = false

            // Open the camera device using the ARCore wrapped callback.
            cameraManager.openCamera(cameraId, wrappedCallback, backgroundHandler)
        } catch (e: CameraAccessException) {
            Log.e(TAG, "Failed to open camera", e)
        } catch (e: IllegalArgumentException) {
            Log.e(TAG, "Failed to open camera", e)
        } catch (e: SecurityException) {
            Log.e(TAG, "Failed to open camera", e)
        }
    }

    private fun <T> checkIfKeyCanCauseDelay(key: CaptureRequest.Key<T>): Boolean {
        if (Build.VERSION.SDK_INT >= 28) {
            // On Android P and later, return true if key is difficult to apply per-frame.
            return keysThatCanCauseCaptureDelaysWhenModified.contains(key)
        } else {
            // On earlier Android versions, log a warning since there is no API to determine whether
            // the key is difficult to apply per-frame. Certain keys such as CONTROL_AE_TARGET_FPS_RANGE
            // are known to cause a noticeable delay on certain devices.
            // If avoiding unexpected capture delays when switching between non-AR and AR modes is
            // important, verify the runtime behavior on each pre-Android P device on which the app will
            // be distributed. Note that this device-specific runtime behavior may change when the
            // device's operating system is updated.
            Log.w(
                TAG,
                "Changing "
                        + key
                        + " may cause a noticeable capture delay. Please verify actual runtime behavior on"
                        + " specific pre-Android P devices that this app will be distributed on."
            )
            // Allow the change since we're unable to determine whether it can cause unexpected delays.
            return false
        }
    }

    // If possible, apply effect in non-AR mode, to help visually distinguish between from AR mode.
    private fun setCameraEffects(captureBuilder: CaptureRequest.Builder?) {
        if (checkIfKeyCanCauseDelay(CaptureRequest.CONTROL_EFFECT_MODE)) {
            Log.w(
                TAG,
                "Not setting CONTROL_EFFECT_MODE since it can cause delays between transitions."
            )
        } else {
            Log.d(TAG, "Setting CONTROL_EFFECT_MODE to SEPIA in non-AR mode.")
            captureBuilder!!.set(
                CaptureRequest.CONTROL_EFFECT_MODE, CaptureRequest.CONTROL_EFFECT_MODE_SEPIA
            )
        }
    }

    // Close the camera device.
    private fun closeCamera() {
        if (::captureSession.isInitialized) {
            captureSession.close()
            // captureSession = null TODO
        }
        if (::cameraDevice.isInitialized) {
            waitUntilCameraCaptureSessionIsActive()
            safeToExitApp.close()
            cameraDevice.close()
            safeToExitApp.block()
        }
        if (::cpuImageReader.isInitialized) {
            cpuImageReader.close()
            //cpuImageReader = null TODO
        }
    }

    // Surface texture on frame available callback, used only in non-AR mode.
    override fun onFrameAvailable(surfaceTexture: SurfaceTexture) {
        // Log.d(TAG, "onFrameAvailable()");
    }

    // CPU image reader callback.
    override fun onImageAvailable(imageReader: ImageReader) {
        val image = imageReader.acquireLatestImage()
        if (image == null) {
            Log.w(TAG, "onImageAvailable: Skipping null image.")
            return
        }
        image.close()
        cpuImagesProcessed++

        // Reduce the screen update to once every two seconds with 30fps if running as automated test.
        if (!automatorRun.get() || automatorRun.get() && cpuImagesProcessed % 60 == 0) {
            runOnUiThread {
                statusTextView.text = ("Poses received: "
                        + myPointCloud.counter
                        +"\nConnection status: "
                        + if(connectionStatus) {"Connected"} else {"Not Connected"}
                        +"\nCurrent speed: "
                        + currentSpeed
                        +"\nCurrent direction: "
                        + currentDirection
                        )
                        /*+ "\n\nMode: "
                        + (if (arMode) "AR" else "non-AR")
                        + " \nARCore active: "
                        + arcoreActive
                        + " \nShould update surface texture: "
                        + shouldUpdateSurfaceTexture.get())*/
            }
        }
    }

    // Android permission request callback.
    override fun onRequestPermissionsResult(
        requestCode: Int,
        permissions: Array<String>,
        results: IntArray
    ) {
        super.onRequestPermissionsResult(requestCode, permissions, results)
        when (requestCode) {
            LOCATION_PERMISSION_REQUEST_CODE -> {
                if (results.firstOrNull() == PackageManager.PERMISSION_DENIED) {
                    requestLocationPermission()
                } else {
                    //TODO start scan
                }
            }
        }

        if (!CameraPermissionHelper.hasCameraPermission(this)) {
            Toast.makeText(
                applicationContext,
                "Camera permission is needed to run this application",
                Toast.LENGTH_LONG
            )
                .show()
            if (!CameraPermissionHelper.shouldShowRequestPermissionRationale(this)) {
                // Permission denied with checking "Do not ask again".
                CameraPermissionHelper.launchPermissionSettings(this)
            }
            finish()
        }
    }

    // Android focus change callback.
    override fun onWindowFocusChanged(hasFocus: Boolean) {
        super.onWindowFocusChanged(hasFocus)
        FullScreenHelper.setFullScreenOnWindowFocusChanged(this, hasFocus)
    }

    // GL surface created callback. Will be called on the GL thread.
    override fun onSurfaceCreated(gl: GL10, config: EGLConfig) {
        surfaceCreated = true

        // Set GL clear color to black.
        GLES20.glClearColor(0f, 0f, 0f, 1.0f)

        // Prepare the rendering objects. This involves reading shaders, so may throw an IOException.
        try {
            // Create the camera preview image texture. Used in non-AR and AR mode.
            backgroundRenderer.createOnGlThread(this)
            planeRenderer.createOnGlThread(this, "models/trigrid.png")
            pointCloudRenderer.createOnGlThread(this)
            myPointCloudRenderer.createOnGlThread(this)
            virtualObject.createOnGlThread(this, "models/andy.obj", "models/andy.png")
            virtualObject.setMaterialProperties(0.0f, 2.0f, 0.5f, 6.0f)
            virtualObjectShadow.createOnGlThread(
                this, "models/andy_shadow.obj", "models/andy_shadow.png"
            )
            virtualObjectShadow.setBlendMode(ObjectRenderer.BlendMode.Shadow)
            virtualObjectShadow.setMaterialProperties(1.0f, 0.0f, 0.0f, 1.0f)
            openCamera()
        } catch (e: IOException) {
            Log.e(TAG, "Failed to read an asset file", e)
        }
    }

    // GL surface changed callback. Will be called on the GL thread.
    override fun onSurfaceChanged(gl: GL10, width: Int, height: Int) {
        GLES20.glViewport(0, 0, width, height)
        displayRotationHelper.onSurfaceChanged(width, height)
        runOnUiThread {
            // Adjust layout based on display orientation.
            imageTextLinearLayout.orientation = if (width > height) LinearLayout.HORIZONTAL else LinearLayout.VERTICAL
        }
    }

    // GL draw callback. Will be called each frame on the GL thread.
    override fun onDrawFrame(gl: GL10) {

        // Use the cGL clear color specified in onSurfaceCreated() to erase the GL surface.
        GLES20.glClear(GLES20.GL_COLOR_BUFFER_BIT or GLES20.GL_DEPTH_BUFFER_BIT)
        if (!shouldUpdateSurfaceTexture.get()) {
            // Not ready to draw.
            return
        }

        // Handle display rotations.
        displayRotationHelper.updateSessionIfNeeded(sharedSession)
        try {
            if (arMode) {
                onDrawFrameARCore()
            } else {
                //onDrawFrameCamera2()
            }
        } catch (t: Throwable) {
            // Avoid crashing the application due to unhandled exceptions.
            Log.e(TAG, "Exception on the OpenGL thread", t)
        }
    }

    // Draw frame when in non-AR mode. Called on the GL thread.
   /* fun onDrawFrameCamera2() {
        val texture = sharedCamera.surfaceTexture

        // ARCore may attach the SurfaceTexture to a different texture from the camera texture, so we
        // need to manually reattach it to our desired texture.
        if (isFirstFrameWithoutArcore.getAndSet(false)) {
            try {
                texture.detachFromGLContext()
            } catch (e: RuntimeException) {
                // Ignore if fails, it may not be attached yet.
            }
            texture.attachToGLContext(backgroundRenderer.getTextureId())
        }

        // Update the surface.
        texture.updateTexImage()

        // Account for any difference between camera sensor orientation and display orientation.
        val rotationDegrees: Int = displayRotationHelper.getCameraSensorToDisplayRotation(cameraId)

        // Determine size of the camera preview image.
        val size = sharedSession.cameraConfig.textureSize

        // Determine aspect ratio of the output GL surface, accounting for the current display rotation
        // relative to the camera sensor orientation of the device.
        val displayAspectRatio: Float =
            displayRotationHelper.getCameraSensorRelativeViewportAspectRatio(cameraId)

        // Render camera preview image to the GL surface.
        backgroundRenderer.draw(size.width, size.height, displayAspectRatio, rotationDegrees)
    }*/

    // Draw frame when in AR mode. Called on the GL thread.
    @Throws(CameraNotAvailableException::class)
    fun onDrawFrameARCore() {
        if (!arcoreActive) {
            // ARCore not yet active, so nothing to draw yet.
            return
        }
        if (errorCreatingSession) {
            // Session not created, so nothing to draw.
            return
        }

        // Perform ARCore per-frame update.
        val frame = sharedSession.update()
        val camera = frame.camera


        progressBar.setProgress(min((myPointCloud.counter/2).toInt(),100),true)


        // Handle screen tap.
        //handleTap(frame, camera)
        if(anchorRequested){
            //addAnchor(camera)
            if (camera.trackingState == TrackingState.TRACKING){
                val objColor = floatArrayOf(66.0f, 133.0f, 244.0f, 255.0f)

                val anchor=sharedSession.createAnchor(camera.pose)

                anchors.add(ColoredAnchor(anchor, objColor))

                compositeCloud=CompositePointCloud(anchor)
            }

            anchorRequested=false
        }

        if(anchors.size>0 && !compositeCloud.lock){
            frame.updatedAnchors.forEach{ anchor->
                if(anchor.equals(compositeCloud.anchor)){
                    compositeCloud.updateAnchorPose()
                }
            }

            val pointSet = mutableListOf<CloudPoint>()

            //Track camera position
            pointSet.add(CloudPoint(0f,0f,0f))


            //add latest lidar points and clear list to avoid duplicates
            lastLidarPoints.forEach { pt ->
                pointSet.add(CloudPoint(pt.toCartesian(90),0f))
            }
            lastLidarPoints.clear()

            compositeCloud.addPointSet(camera.pose,pointSet)

            if(cpuImagesProcessed.mod(10)==0){//avoid adding too many points
                compositeCloud.addDepthPoints(frame, camera.pose)
            }

            /*val cloud=frame.acquirePointCloud()
            val cloudPoints=cloud.points
            cloud.close()
            try{
               while(cloudPoints.hasRemaining()){
                    compositeCloud.arCloudPoints.add(CloudPoint(cloudPoints.get(),cloudPoints.get(),cloudPoints.get(),cloudPoints.get()))
                   compositeCloud.numPoints++
                }
            }catch(e: BufferUnderflowException){}*/

        }

        if(myPointCloud.counter.mod(500)==0 && anchors.size>0){//every 500 frames save current points and clear
            saveToPLY()

            compositeCloud.clear()
        }

        //Add lidar points to cloud TODO fix orientation issue
        if(addToCloudRequested && anchors.size>0){
            val pointList= mutableListOf<CloudPoint>()
            myPointCloud.pointArray.forEach {pt->
                if(pt.distance>0){ //TODO check intensity
                    pointList.add(CloudPoint(pt.toCartesian(myPointCloud.angleOffset)))
                }
            }
            compositeCloud.addPointSet(camera.pose,pointList)

            addToCloudRequested=false
        }

        if(establishCloudAnchor && anchors.size>0 && myPointCloud.counter>=1000){
            if(::cloudAnchor.isInitialized){
                if(myPointCloud.counter.mod(10)==0){//to avoid too many messages only output every 10 frames
                    Log.d("ASD","Establish CloudAnchorState: ${cloudAnchor.cloudAnchorState}")
                    if(cloudAnchor.cloudAnchorState==Anchor.CloudAnchorState.SUCCESS){//cloud anchor established successfully
                        Log.d("ASD","Cloud Anchor Id: ${cloudAnchor.cloudAnchorId}")
                        if(establishCloudAnchor){
                            saveCloudAnchorId(cloudAnchor.cloudAnchorId)
                            establishCloudAnchor=false
                        }
                    }
                }
            }else{
                cloudAnchor=sharedSession.hostCloudAnchor(anchors[0].anchor)
            }
        }

        if(resolveCloudAnchor && anchors.size>0 && myPointCloud.counter>=1000){
            if(::cloudAnchor.isInitialized){
                if(myPointCloud.counter.mod(10)==0) {//to avoid too many messages only output every 10 frames
                    Log.d("ASD","Resolve CloudAnchorState: ${cloudAnchor.cloudAnchorState}")
                }

                if(cloudAnchor.cloudAnchorState==Anchor.CloudAnchorState.SUCCESS){//cloud anchor resolved successfully
                    val objColor = floatArrayOf(255.0f, 10.0f, 10.0f, 255.0f)
                    anchors.add(ColoredAnchor(cloudAnchor, objColor))
                }
            }else{
                cloudAnchor=sharedSession.resolveCloudAnchor(CLOUD_ANCHOR_ID)
            }
        }

        // If frame is ready, render camera preview image to the GL surface.
        backgroundRenderer.draw(frame)

        // Keep the screen unlocked while tracking, but allow it to lock when tracking stops.
        trackingStateHelper.updateKeepScreenOnFlag(camera.trackingState)

        // If not tracking, don't draw 3D objects.
        if (camera.trackingState == TrackingState.PAUSED) {
            return
        }

        // Get projection matrix.
        val projmtx = FloatArray(16)
        camera.getProjectionMatrix(projmtx, 0, 0.1f, 100.0f)

        // Get camera matrix and draw.
        val viewmtx = FloatArray(16)
        camera.getViewMatrix(viewmtx, 0)

        // Compute lighting from average intensity of the image.
        // The first three components are color scaling factors.
        // The last one is the average pixel intensity in gamma space.
        val colorCorrectionRgba = FloatArray(4)
        frame.lightEstimate.getColorCorrection(colorCorrectionRgba, 0)
        frame.acquirePointCloud().use { pointCloud ->
            pointCloudRenderer.update(pointCloud)
            pointCloudRenderer.draw(viewmtx, projmtx)
        }

        //TODO mostra una serie di punti lidar finti per ora
        if(myPointCloud.pointList.size==360 || true){
            myPointCloud.lastTimestamp=frame.timestamp
            myPointCloud.setPose(camera.pose)

            myPointCloudRenderer.update(myPointCloud)
            myPointCloudRenderer.draw(viewmtx, projmtx)
        }else{
            //Log.d("ASD","ASD no 360")
        }

       // val depthImage=frame.acquireDepthImage()
        //processDepthImage(depthImage)



        // If we detected any plane and snackbar is visible, then hide the snackbar.
        if (messageSnackbarHelper.isShowing()) {
            for (plane: Plane in sharedSession.getAllTrackables(
                Plane::class.java
            )) {
                if (plane.trackingState == TrackingState.TRACKING) {
                    messageSnackbarHelper.hide(this)
                    break
                }
            }
        }

        // Visualize planes.
        planeRenderer.drawPlanes(
            sharedSession.getAllTrackables(Plane::class.java), camera.displayOrientedPose, projmtx
        )

        // Visualize anchors created by touch.
        val scaleFactor = 1.0f
        for (coloredAnchor: ColoredAnchor in anchors) {
            if (coloredAnchor.anchor.trackingState != TrackingState.TRACKING) {
                continue
            }
            // Get the current pose of an Anchor in world space. The Anchor pose is updated
            // during calls to sharedSession.update() as ARCore refines its estimate of the world.
            coloredAnchor.anchor.pose.toMatrix(anchorMatrix, 0)

            // Update and draw the model and its shadow.
            virtualObject.updateModelMatrix(anchorMatrix, scaleFactor)
            virtualObjectShadow.updateModelMatrix(anchorMatrix, scaleFactor)
            virtualObject.draw(viewmtx, projmtx, colorCorrectionRgba, coloredAnchor.color)
            virtualObjectShadow.draw(viewmtx, projmtx, colorCorrectionRgba, coloredAnchor.color)
        }



    }


    fun saveToPLY(){

        saveCounter++

        val sdf = SimpleDateFormat("ddMM_hhmm")
        val currentDate = sdf.format(Date())

        /*var output="ply\n" +
                "format ascii 1.0\n" +
                "element vertex ${myPointCloud.posedPointList.size/3}\n" +
                "property float x\n" +
                "property float y\n" +
                "property float z\n" +
                "end_header\n"

        var i=0
        while(i<myPointCloud.posedPointList.size){
            //X
            output+=myPointCloud.posedPointList[i]
            output+=" "
            i++
            //Y
            output+=myPointCloud.posedPointList[i]
            output+=" "
            i++
            //Z
            output+=myPointCloud.posedPointList[i]
            output+=" "
            i++

            output+="\n"
        }*/





        val file = File(this.filesDir, "cloud${currentDate}_$saveCounter.ply")
        val fileWriter= FileWriter(file,false)

        var str=compositeCloud.toPLY()



        try{
            fileWriter.write(str)

        }catch(e:Exception){
            Log.d("KEK","KEK $e")
        }finally{
            fileWriter.close()
        }


    }

    fun saveCloudAnchorId(id:String){
        val sdf = SimpleDateFormat("ddMM_hhmm")
        val currentDate = sdf.format(Date())
        val file = File(this.filesDir, "cloudAnchor${currentDate}.ply")
        val fileWriter= FileWriter(file,false)

        val str="cloudAnchor${currentDate} Id: $id"

        try{
            fileWriter.write(str)

        }catch(e:Exception){
            Log.d("KEK","KEK $e")
        }finally{
            fileWriter.close()
        }
    }


    fun readFromFile(){
        val file = File(this.filesDir, "prova.txt")
        val fileReader= FileReader(file)
        //val cb= CharBuffer.allocate(100)

        val cb=CharArray(file.length().toInt())

        try{
            fileReader.read(cb)
            Log.d("KEK","KEK ${cb.joinToString("")}")
        }catch(e:Exception){
            Log.d("KEK","KEK $e")
        }finally{
            fileReader.close()
        }
    }

    fun addAnchor(camera:Camera){
        if (camera.trackingState == TrackingState.TRACKING){
            // Hits are sorted by depth. Consider only closest hit on a plane or oriented point.
            // Cap the number of objects created. This avoids overloading both the
            // rendering system and ARCore.
            if (anchors.size >= 20) {
                anchors[0].anchor.detach()
                anchors.removeAt(0)
            }

            // Assign a color to the object for rendering based on the trackable type
            // this anchor attached to. For AR_TRACKABLE_POINT, it's blue color, and
            // for AR_TRACKABLE_PLANE, it's green color.
            val objColor = floatArrayOf(66.0f, 133.0f, 244.0f, 255.0f)

            // Adding an Anchor tells ARCore that it should track this position in
            // space. This anchor is created on the Plane to place the 3D model
            // in the correct position relative both to the world and to the plane.
            anchors.add(ColoredAnchor(sharedSession.createAnchor(camera.displayOrientedPose), objColor))
        }
    }

    // Handle only one tap per frame, as taps are usually low frequency compared to frame rate.
    /*private fun handleTap(frame: Frame, camera: Camera) {
        val tap: MotionEvent? = tapHelper.poll()
        if (tap != null && camera.trackingState == TrackingState.TRACKING) {
            for (hit: HitResult in frame.hitTest(tap)) {
                // Check if any plane was hit, and if it was hit inside the plane polygon
                val trackable = hit.trackable
                // Creates an anchor if a plane or an oriented point was hit.
                if ((trackable is Plane
                            && trackable.isPoseInPolygon(hit.hitPose)
                            && PlaneRenderer.calculateDistanceToPlane(hit.hitPose, camera.pose) > 0)
                    || (trackable is Point
                            && trackable.orientationMode
                            == Point.OrientationMode.ESTIMATED_SURFACE_NORMAL)
                ) {
                    // Hits are sorted by depth. Consider only closest hit on a plane or oriented point.
                    // Cap the number of objects created. This avoids overloading both the
                    // rendering system and ARCore.
                    if (anchors.size >= 20) {
                        anchors[0].anchor.detach()
                        anchors.removeAt(0)
                    }

                    // Assign a color to the object for rendering based on the trackable type
                    // this anchor attached to. For AR_TRACKABLE_POINT, it's blue color, and
                    // for AR_TRACKABLE_PLANE, it's green color.
                    val objColor: FloatArray
                    if (trackable is Point) {
                        objColor = floatArrayOf(66.0f, 133.0f, 244.0f, 255.0f)
                    } else if (trackable is Plane) {
                        objColor = floatArrayOf(139.0f, 195.0f, 74.0f, 255.0f)
                    } else {
                        objColor = DEFAULT_COLOR
                    }

                    // Adding an Anchor tells ARCore that it should track this position in
                    // space. This anchor is created on the Plane to place the 3D model
                    // in the correct position relative both to the world and to the plane.
                    anchors.add(ColoredAnchor(hit.createAnchor(), objColor))
                    break
                }
            }
        }
    }*//*userRequestedInstall=*/// Request ARCore installation or update if needed.

    // Make sure ARCore is installed and supported on this device.
    private val isARCoreSupportedAndUpToDate: Boolean
        get() {
            // Make sure ARCore is installed and supported on this device.
            when (val availability = ArCoreApk.getInstance().checkAvailability(this)) {
                Availability.SUPPORTED_INSTALLED -> {
                }
                Availability.SUPPORTED_APK_TOO_OLD, Availability.SUPPORTED_NOT_INSTALLED -> try {
                    // Request ARCore installation or update if needed.
                    when (ArCoreApk.getInstance().requestInstall(this,  /*userRequestedInstall=*/true)) {
                        InstallStatus.INSTALL_REQUESTED -> {
                            Log.e(TAG, "ARCore installation requested.")
                            return false
                        }
                        InstallStatus.INSTALLED -> {}
                        else -> {}
                    }
                } catch (e: UnavailableException) {
                    Log.e(TAG, "ARCore not installed", e)
                    runOnUiThread {
                        Toast.makeText(applicationContext,"ARCore not installed\n$e",Toast.LENGTH_LONG).show()
                    }
                    finish()
                    return false
                }
                Availability.UNKNOWN_ERROR, Availability.UNKNOWN_CHECKING, Availability.UNKNOWN_TIMED_OUT, Availability.UNSUPPORTED_DEVICE_NOT_CAPABLE -> {
                    Log.e(
                        TAG,
                        "ARCore is not supported on this device, ArCoreApk.checkAvailability() returned "
                                + availability
                    )
                    runOnUiThread {
                        Toast.makeText(applicationContext, "ARCore is not supported on this device, ArCoreApk.checkAvailability() returned $availability", Toast.LENGTH_LONG).show()
                    }
                    return false
                }
            }
            return true
        }

    //check if we have a certain permission
    private fun Context.hasPermission(permissionType: String): Boolean {
        return ContextCompat.checkSelfPermission(this, permissionType) ==
                PackageManager.PERMISSION_GRANTED
    }

    //to scan for devices the fine location permission is required, prompt the user to give location permission
    private fun requestLocationPermission() {
        if (isLocationPermissionGranted) {
            return
        }
        runOnUiThread {
            AlertDialog.Builder(this)
                .setTitle("Location permission required")
                .setMessage("Starting from Android M (6.0), the system requires apps to be granted " +
                        "location access in order to scan for BLE devices.")
                .setPositiveButton(android.R.string.ok){ _: DialogInterface, _: Int ->
                    requestPermission(
                        Manifest.permission.ACCESS_FINE_LOCATION,
                        LOCATION_PERMISSION_REQUEST_CODE
                    )
                }
                .show()
        }
    }


    //request a specified permission
    private fun Activity.requestPermission(permission: String, requestCode: Int) {
        ActivityCompat.requestPermissions(this, arrayOf(permission), requestCode)
    }

    companion object {
        private val TAG = MainActivity::class.java.simpleName
        private val DEFAULT_COLOR = floatArrayOf(0f, 0f, 0f, 0f)

        // Required for test run.
        private const val AUTOMATOR_DEFAULT: Short = 0
        private const val AUTOMATOR_KEY = "automator"
    }
}
