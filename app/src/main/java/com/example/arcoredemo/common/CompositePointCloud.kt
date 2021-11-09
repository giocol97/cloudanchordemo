package com.example.arcoredemo.common

import android.graphics.*
import android.media.Image
import android.opengl.Matrix
import android.util.Log
import com.google.ar.core.Anchor
import com.google.ar.core.CameraIntrinsics
import com.google.ar.core.Frame
import com.google.ar.core.Pose
import com.google.ar.core.exceptions.NotYetAvailableException
import java.io.ByteArrayOutputStream
import java.nio.ByteBuffer
import java.nio.ByteOrder
import java.nio.FloatBuffer
import java.nio.ShortBuffer
import kotlin.experimental.and
import kotlin.system.measureTimeMillis


const val FLOATS_PER_POINT = 4 // X,Y,Z,confidence.

class CompositePointCloud(val anchor: Anchor){//:CloudInterface { TODO possibile integrare con cloud renderer


    val points= mutableListOf<Pair<Pose,List<CloudPoint>>>()

    val worldPoints=mutableListOf<CloudPoint>()

    //val arCloudPoints=mutableListOf<CloudPoint>()

    var numPoints=0

    var composedAnchorPose=anchor.pose

    var lock=false

    fun addPointSet(p:Pose,set:List<CloudPoint>){
        points.add(Pair(p,set))
        numPoints+=set.size
    }

    fun updateAnchorPose(){
        composedAnchorPose=anchor.pose.compose(composedAnchorPose)
    }

    //empty all point lists
    fun clear(){
        points.clear()
        worldPoints.clear()
        numPoints=0
    }

    fun toPLY():String{
        val output=StringBuilder()
        output.append("ply\n" +
                "format ascii 1.0\n" +
                "element vertex ${numPoints}\n" +
                "property float x\n" +
                "property float y\n" +
                "property float z\n" +
                "property uchar red\n" +
                "property uchar green\n" +
                "property uchar blue\n" +
                "property float confidence\n" +
                "end_header\n")

        try{

            points.forEach { posePoints->
                val pose=composedAnchorPose.compose(posePoints.first)//.extractTranslation())

                posePoints.second.forEach{ pt ->
                    val arr=pose.transformPoint(pt.toArray())
                    if(pt.confidence==0f){
                        output.append("${arr[0]} ${arr[1]} ${arr[2]} 0 0 255 ${1.0f}\n")
                    }else{
                        output.append("${arr[0]} ${arr[1]} ${arr[2]} 255 0 0 ${1.0f}\n")
                    }

                }
            }

            worldPoints.forEach {pt ->
                output.append("${pt.x} ${pt.y} ${pt.z} ${pt.r} ${pt.g} ${pt.b} ${pt.confidence}\n")
            }

            /*arCloudPoints.forEach { pt->
                output.append("${pt.x} ${pt.y} ${pt.z} 0 255 0 ${pt.confidence}\n")
            }*/

        }catch(e:Exception){
            Log.d("ASD","ASD $e")
        }
        return output.toString()
    }



    fun addDepthPoints(frame: Frame, cameraPose: Pose) {
        try {
            val depthImage: Image = frame.acquireRawDepthImage()
            val confidenceImage: Image = frame.acquireRawDepthConfidenceImage()

            // Retrieve the intrinsic camera parameters corresponding to the depth image to
            // transform 2D depth pixels into 3D points. See more information about the depth values
            // at
            // https://developers.google.com/ar/develop/java/depth/overview#understand-depth-values.
            val intrinsics: CameraIntrinsics = frame.getCamera().getTextureIntrinsics()
            val modelMatrix = FloatArray(16)
            cameraPose.toMatrix(modelMatrix, 0)

            val colorImage=frame.acquireCameraImage()
            val bytes= imageToByteArray(colorImage)
            colorImage.close()
            var colorImageBitmap= BitmapFactory.decodeByteArray(bytes,0, bytes!!.size)
            colorImageBitmap= Bitmap.createScaledBitmap(colorImageBitmap,depthImage.width,depthImage.height,false)
            //colorImageBitmap=colorImageBitmap.rotate(90f)

            val worldPointsAdded = convertRawDepthImagesTo3dPointBuffer(
                depthImage, confidenceImage, intrinsics, modelMatrix, colorImageBitmap
            )

            numPoints+=worldPointsAdded
            depthImage.close()
            confidenceImage.close()

        } catch (e: NotYetAvailableException) {
            Log.d("ASD", "ASD ${e.message}")
            // This normally means that depth data is not available yet.
            // This is normal, so you don't have to spam the logcat with this.
        }
    }

    /*private fun imageToByteArray(image: Image): ByteArray {
        val data: ByteArray
        val buffer0: ByteBuffer = image.planes[0].buffer
        val buffer0_size: Int = buffer0.remaining()
        data = ByteArray(buffer0_size)
        buffer0.get(data, 0, buffer0_size)
        return data
    }*/

    private fun Bitmap.rotate(degrees: Float): Bitmap {
        val matrix = android.graphics.Matrix().apply { postRotate(degrees) }
        return Bitmap.createBitmap(this, 0, 0, width, height, matrix, true)
    }

    /** Apply camera intrinsics to convert depth image into a 3D pointcloud.  */
    private fun convertRawDepthImagesTo3dPointBuffer(
        depth: Image,
        confidence: Image,
        cameraTextureIntrinsics: CameraIntrinsics,
        modelMatrix: FloatArray,
        colorImage:Bitmap
    ): Int {
        // Java uses big endian so change the endianness to ensure
        // that the depth data is in the correct byte order.
        val depthImagePlane = depth.planes[0]
        val depthByteBufferOriginal: ByteBuffer = depthImagePlane.buffer
        val depthByteBuffer: ByteBuffer = ByteBuffer.allocate(depthByteBufferOriginal.capacity())
        depthByteBuffer.order(ByteOrder.LITTLE_ENDIAN)
        while (depthByteBufferOriginal.hasRemaining()) {
            depthByteBuffer.put(depthByteBufferOriginal.get())
        }
        depthByteBuffer.rewind()
        val depthBuffer: ShortBuffer = depthByteBuffer.asShortBuffer()
        val confidenceImagePlane = confidence.planes[0]
        val confidenceBufferOriginal: ByteBuffer = confidenceImagePlane.buffer
        val confidenceBuffer: ByteBuffer = ByteBuffer.allocate(confidenceBufferOriginal.capacity())
        confidenceBuffer.order(ByteOrder.LITTLE_ENDIAN)
        while (confidenceBufferOriginal.hasRemaining()) {
            confidenceBuffer.put(confidenceBufferOriginal.get())
        }
        confidenceBuffer.rewind()

        // To transform 2D depth pixels into 3D points, retrieve the intrinsic camera parameters
        // corresponding to the depth image. See more information about the depth values at
        // https://developers.google.com/ar/develop/java/depth/overview#understand-depth-values.
        val intrinsicsDimensions = cameraTextureIntrinsics.imageDimensions
        val depthWidth = depth.width
        val depthHeight = depth.height
        val fx = cameraTextureIntrinsics.focalLength[0] * depthWidth / intrinsicsDimensions[0]
        val fy = cameraTextureIntrinsics.focalLength[1] * depthHeight / intrinsicsDimensions[1]
        val cx = cameraTextureIntrinsics.principalPoint[0] * depthWidth / intrinsicsDimensions[0]
        val cy = cameraTextureIntrinsics.principalPoint[1] * depthHeight / intrinsicsDimensions[1]

        // Allocate the destination point buffer. If the number of depth pixels is larger than
        // `maxNumberOfPointsToRender` we uniformly subsample. The raw depth image may have
        // different resolutions on different devices.
        val maxNumberOfPointsToRender = 20000f
        val step =
            Math.ceil(Math.sqrt((depthWidth * depthHeight / maxNumberOfPointsToRender).toDouble()))
                .toInt()
        //val points = FloatBuffer.allocate(depthWidth / step * depthHeight / step * FLOATS_PER_POINT)
        val pointCamera = FloatArray(4)
        val pointWorld = FloatArray(4)
        var y = 0
        var cont=0
        var pixel=0
        while (y < depthHeight) {
            var x = 0
            while (x < depthWidth) {

                // Depth images are tightly packed, so it's OK to not use row and pixel strides.
                val depthMillimeters: Int =
                    depthBuffer.get(y * depthWidth + x).toInt() // Depth image pixels are in mm.
                if (depthMillimeters == 0) {
                    // Pixels with value zero are invalid, meaning depth estimates are missing from
                    // this location.
                    x += step
                    continue
                }
                val depthMeters = depthMillimeters / 1000.0f // Depth image pixels are in mm.

                // Retrieve the confidence value for this pixel.
                val confidencePixelValue: Byte = confidenceBuffer.get(
                    y * confidenceImagePlane.rowStride
                            + x * confidenceImagePlane.pixelStride
                )
                val confidenceNormalized = ((confidencePixelValue and 0xff.toByte())) / 255.0f

                // Unproject the depth into a 3D point in camera coordinates.
                pointCamera[0] = depthMeters * (x - cx) / fx
                pointCamera[1] = depthMeters * (cy - y) / fy
                pointCamera[2] = -depthMeters
                pointCamera[3] = 1f

                // Apply model matrix to transform point into world coordinates.
                Matrix.multiplyMV(pointWorld, 0, modelMatrix, 0, pointCamera, 0)
                /*points.put(pointWorld[0]) // X.
                points.put(pointWorld[1]) // Y.
                points.put(pointWorld[2]) // Z.
                points.put(confidenceNormalized)*/
                if(confidenceNormalized>0.1f){
                    pixel=colorImage.getPixel(x,y)

                    worldPoints.add(CloudPoint(pointWorld,confidenceNormalized,
                        Color.red(pixel),Color.green(pixel),Color.blue(pixel)))

                    cont++
                }
                x += step
            }
            y += step
        }
        //points.rewind()
        //return points
        return cont
    }

    fun imageToByteArray(image: Image): ByteArray? {
        var data: ByteArray? = null
        if (image.format == ImageFormat.JPEG) {
            val planes = image.planes
            val buffer = planes[0].buffer
            data = ByteArray(buffer.capacity())
            buffer[data]
            return data
        } else if (image.format == ImageFormat.YUV_420_888) {
            data = NV21toJPEG(
                YUV_420_888toNV21(image),
                image.width, image.height
            )
        }
        return data
    }

    private fun YUV_420_888toNV21(image: Image): ByteArray {
        val nv21: ByteArray
        val yBuffer = image.planes[0].buffer
        val vuBuffer = image.planes[2].buffer
        val ySize = yBuffer.remaining()
        val vuSize = vuBuffer.remaining()
        nv21 = ByteArray(ySize + vuSize)
        yBuffer[nv21, 0, ySize]
        vuBuffer[nv21, ySize, vuSize]
        return nv21
    }

    private fun NV21toJPEG(nv21: ByteArray, width: Int, height: Int): ByteArray {
        val out = ByteArrayOutputStream()
        val yuv = YuvImage(nv21, ImageFormat.NV21, width, height, null)
        yuv.compressToJpeg(Rect(0, 0, width, height), 100, out)
        return out.toByteArray()
    }


}


